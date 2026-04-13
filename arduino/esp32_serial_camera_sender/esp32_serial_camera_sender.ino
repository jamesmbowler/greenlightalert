#include "esp_camera.h"
#include "img_converters.h"
#include "esp32-hal-log.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <SPI.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define EI_CLASSIFIER_TFLITE_ENABLE_ESP_NN 1
#define EI_MAX_OVERFLOW_BUFFER_COUNT 50
#include <ESP32_OV5640_AF.h>
#include <jbowler-project-1_inferencing.h>
#include "camera_pins.h"
#include "config.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "red_light_box_shared.h"
#include "traffic_light_detector_shared.h"

namespace {

constexpr char kFirmwareVersion[] = "trafficcam-debug-2026-03-16-1";
constexpr char kLogTag[] = "TrafficCam";
constexpr size_t kMaxSerializedBoxes = 10;
constexpr size_t kMaxTrackedRedPixels = 2048;
constexpr int kPatchSignatureSize = 16;
constexpr char kBleServiceUuid[] = "7A1C0000-0C4E-4F67-8D80-61A2E5B10000";
constexpr char kBleStatusCharacteristicUuid[] = "7A1C0001-0C4E-4F67-8D80-61A2E5B10000";
constexpr char kBleImageMetadataCharacteristicUuid[] = "7A1C0002-0C4E-4F67-8D80-61A2E5B10000";
constexpr char kBleImageDataCharacteristicUuid[] = "7A1C0003-0C4E-4F67-8D80-61A2E5B10000";
constexpr char kBleControlCharacteristicUuid[] = "7A1C0004-0C4E-4F67-8D80-61A2E5B10000";

OV5640 ov5640;

struct SerializableBoundingBox {
  char label[32] = {0};
  int rawX = 0;
  int rawY = 0;
  int rawWidth = 0;
  int rawHeight = 0;
  int xMin = 0;
  int yMin = 0;
  int xMax = 0;
  int yMax = 0;
  int clusterCenterX = 0;
  int clusterCenterY = 0;
  float confidence = 0.0f;
};

struct TrafficLightState {
  bool redFound = false;
  bool redLocked = false;
  bool redOn = false;
  bool redFoundByEi = false;
  bool hasEiCrop = false;
  float eiConfidence = 0.0f;
  int eiCropX = 0;
  int eiCropY = 0;
  int eiCropWidth = 0;
  int eiCropHeight = 0;
  float eiScale = 1.0f;
  int eiResizedWidth = 0;
  int eiResizedHeight = 0;
  int eiInputCropX = 0;
  int eiInputCropY = 0;
  std::string rawEiOutput;
  traffic::LampDetection red{};
  size_t serializedBoxCount = 0;
  SerializableBoundingBox serializedBoxes[kMaxSerializedBoxes]{};
};

struct __attribute__((packed)) ImagePacketHeader {
  char magic[4];
  uint8_t version;
  uint8_t imageType;
  uint16_t width;
  uint16_t height;
  uint32_t sequence;
  uint32_t payloadSize;
};

enum ImageType : uint8_t {
  kImageTypeRedFrame = 1,
  kImageTypePeriodicFrame = 3,
  kImageTypeRedFrameBoxes = 4,
  kImageTypeRedFrameEi = 5,
  kImageTypeEiModelInput = 6,
  kImageTypeEiCropRoi = 7,
  kImageTypeGreenLight = 8,
  kImageTypeGreenLightSearch = 9,
  kImageTypeSceneChanged = 10,
  kImageTypeRedLightBox = 11,
  kImageTypeRawEiOutput = 12,
};

uint32_t gPacketSequence = 0;
int gStableRedFrames = 0;
int gLastRedCenterX = -1000;
int gLastRedCenterY = -1000;
bool gRedSnapshotLatched = false;
uint32_t gLastSnapshotMs = 0;
bool gHasLockedTarget = false;
traffic::LampDetection gLockedRed{};
int gRedLockMissingFrames = 0;
SemaphoreHandle_t gCameraMutex = nullptr;
bool gHasPsram = false;
uint32_t gLoopCounter = 0;
uint32_t gFrameCounter = 0;
uint32_t gCaptureFailureCount = 0;
uint32_t gLastDetectorRunMs = 0;
uint8_t* gEiInputBuffer = nullptr;
int gCurrentAeLevel = kAeLevel;
bool gCameraInitialized = false;
bool gGreenWatchLowResolutionMode = false;
bool gOv5640RawWindowRuntimeEnabled = kEnableOv5640RawWindow;
framesize_t gCurrentFrameSize = FRAMESIZE_INVALID;
BLEServer* gBleServer = nullptr;
BLECharacteristic* gBleStatusCharacteristic = nullptr;
BLECharacteristic* gBleImageMetadataCharacteristic = nullptr;
BLECharacteristic* gBleImageDataCharacteristic = nullptr;
BLECharacteristic* gBleControlCharacteristic = nullptr;
bool gBleClientConnected = false;
bool gBleLatestImageRequested = false;
bool gBleSendWindowRequested = false;
uint16_t gBleSendWindowSequence = 0;
bool gPaused = false;
std::string gLastBleStatusPayload;
uint32_t gLastBleStatusSentAtMs = 0;
uint32_t gLastBleHeartbeatLogMs = 0;
uint8_t gCurrentBleStatusCode = 6;
bool gCurrentBleStatusRedFound = false;
bool gCurrentBleStatusGreenOn = false;
bool gHasCurrentBleStatus = false;
const uint8_t* gBleActiveImageData = nullptr;
size_t gBleActiveImageSize = 0;
uint16_t gBleActiveImageWidth = 0;
uint16_t gBleActiveImageHeight = 0;
uint8_t gBleActiveImageReasonCode = 0;
uint32_t gBleActiveImageCapturedAtMs = 0;
uint32_t gBleActiveImageSequence = 0;

void clearBleActiveImage();
void notifyBleImageChunks(const uint8_t* data, size_t size, uint16_t startSequence = 0);
void setCurrentBleStatus(const char* state, const char* summary, bool redFound, bool greenOn,
                         uint32_t nowMs, bool forceNotify = false);
void stopGreenWatch(const char* reason);

uint8_t* createBleJpegRgb888BufferFromEiInput(size_t pixelCount);
uint8_t bma400ReadRegister(uint8_t reg);
void bma400ReadRegisters(uint8_t startReg, uint8_t* data, size_t length);
void bma400WriteRegister(uint8_t reg, uint8_t value);
bool setupBma400();
void logBma400MotionData(uint32_t nowMs);
unsigned bleImageTotalChunks(size_t size);
unsigned bleImageWindowChunkCount(size_t size);

class TrafficCamBleStatusCallbacks : public BLECharacteristicCallbacks {
 public:
  void onStatus(BLECharacteristic* characteristic, Status s, uint32_t code) override {
    log_d("BLE status char callback uuid=%s status=%u code=%lu len=%u",
          characteristic != nullptr ? characteristic->getUUID().toString().c_str() : "<null>",
          static_cast<unsigned>(s), static_cast<unsigned long>(code),
          characteristic != nullptr ? static_cast<unsigned>(characteristic->getLength()) : 0U);
  }

  void onSubscribe(BLECharacteristic* characteristic, ble_gap_conn_desc* desc, uint16_t subValue) override {
    (void)desc;
    log_d("BLE subscribe uuid=%s subValue=%u",
          characteristic != nullptr ? characteristic->getUUID().toString().c_str() : "<null>",
          static_cast<unsigned>(subValue));
  }
};

struct EdgeImpulseTransform {
  int roiX = 0;
  int roiY = 0;
  int roiWidth = 0;
  int roiHeight = 0;
  float scale = 1.0f;
  int resizedWidth = 0;
  int resizedHeight = 0;
  int cropX = 0;
  int cropY = 0;
};

bool prepareEdgeImpulseInput(const camera_fb_t* fb, EdgeImpulseTransform& transform);
framesize_t defaultCameraFrameSize();
bool frameSizeToResolution(framesize_t frameSize, int& width, int& height);
void dumpOv5640WindowRegs(sensor_t* sensor, const char* label);
void dumpOv5640PresetRegisters(sensor_t* sensor, const char* label);
bool writeOv5640Reg16(sensor_t* sensor, int highReg, int lowReg, int value);
camera_fb_t* captureFrameLocked();
void releaseFrameLocked(camera_fb_t* fb);
bool reinitializeCameraForFrameSize(framesize_t frameSize, const char* reason);
bool enterGreenWatchLowResolutionMode(uint32_t nowMs);
bool exitGreenWatchLowResolutionMode();
bool applyOv5640RawWindow(sensor_t* sensor);
bool applyOv5640SensorDigitalZoom(sensor_t* sensor);
void clearInferenceBuffers();

struct JpegCountContext {
  size_t totalBytes = 0;
};

struct JpegSerialWriteContext {
  size_t totalBytes = 0;
};

struct TrackedRedPixel {
  uint16_t x = 0;
  uint16_t y = 0;
};

struct GreenWatchState {
  bool active = false;
  uint32_t startedAtMs = 0;
  uint32_t lastCheckAtMs = 0;
  uint32_t sceneCheckReadyAtMs = 0;
  int frameWidth = 0;
  int frameHeight = 0;
  traffic::LampDetection red{};
  int baselineCenterLuma = 0;
  int baselineCenterR = 0;
  int baselineCenterG = 0;
  int baselineCenterB = 0;
  int coreX = 0;
  int coreY = 0;
  int coreWidth = 0;
  int coreHeight = 0;
  size_t trackedPixelCount = 0;
  TrackedRedPixel trackedPixels[kMaxTrackedRedPixels]{};
  int patchX = 0;
  int patchY = 0;
  int patchWidth = 0;
  int patchHeight = 0;
  uint8_t patchSignature[kPatchSignatureSize * kPatchSignatureSize]{};
};

GreenWatchState gGreenWatch{};

constexpr int kBma400SpiSckPin = 7;
constexpr int kBma400SpiMisoPin = 8;
constexpr int kBma400SpiMosiPin = 9;
constexpr int kBma400SpiCsPin = 2;
constexpr uint32_t kBma400SpiHz = 100000;
constexpr uint8_t kBma400ChipIdReg = 0x00;
constexpr uint8_t kBma400AccXlsbReg = 0x04;
constexpr uint8_t kBma400AccConfig0Reg = 0x19;
constexpr uint8_t kBma400ExpectedChipId = 0x90;
constexpr uint8_t kBma400NormalMode = 0x02;
constexpr uint32_t kBma400LogIntervalMs = 500;

SPIClass gBma400Spi(FSPI);
bool gBma400Ready = false;
uint32_t gLastBma400LogMs = 0;

enum class GreenWatchUpdateResult : uint8_t {
  kNone = 0,
  kRedOffDetected = 1,
  kSceneChanged = 2,
};

class TrafficCamBleServerCallbacks : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer* server) override {
    if (server != nullptr) {
      server->getAdvertising()->stop();
    }
    gBleClientConnected = true;
    log_i("BLE client connected at ms=%lu", static_cast<unsigned long>(millis()));
    log_i("BLE advertising stopped while connected");
  }

  void onDisconnect(BLEServer* server) override {
    (void)server;
    gBleClientConnected = false;
    gBleSendWindowRequested = false;
    log_i("BLE client disconnected at ms=%lu", static_cast<unsigned long>(millis()));
    clearBleActiveImage();
    BLEDevice::startAdvertising();
    log_i("BLE advertising restarted after disconnect");
  }
};

class TrafficCamBleControlCallbacks : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic* characteristic) override {
    if (characteristic == nullptr) {
      return;
    }
    const std::string value = characteristic->getValue().c_str();
    if (value.find("request_latest_image") != std::string::npos) {
      if (gPaused) {
        log_i("BLE control request_latest_image ignored while paused");
      } else {
        gBleLatestImageRequested = true;
        log_i("BLE control request_latest_image");
      }
    }
    if (value.find("pause") != std::string::npos) {
      gPaused = true;
      gBleLatestImageRequested = false;
      gBleSendWindowRequested = false;
      stopGreenWatch("paused");
      setCurrentBleStatus("Paused", "Paused. Detection and image capture are suspended.", false,
                          false, millis(), true);
      log_i("BLE control pause");
    }
    if (value.find("resume") != std::string::npos) {
      gPaused = false;
      setCurrentBleStatus("Monitoring", "Scanning for a red light.", false, false, millis(), true);
      log_i("BLE control resume");
    }
    const bool resendRequested = value.find("\"command\":\"resend_from\"") != std::string::npos;
    const bool continueRequested = value.find("\"command\":\"continue_from\"") != std::string::npos;
    if (resendRequested || continueRequested) {
      const std::string sequenceKey = "\"sequence\":";
      const size_t sequencePos = value.find(sequenceKey);
      if (sequencePos != std::string::npos) {
        const size_t numberStart = sequencePos + sequenceKey.size();
        const uint16_t sequence =
            static_cast<uint16_t>(std::strtoul(value.c_str() + numberStart, nullptr, 10));
        gBleSendWindowSequence = sequence;
        gBleSendWindowRequested = true;
        log_d("BLE control %s sequence=%u",
              resendRequested ? "resend_from" : "continue_from",
              static_cast<unsigned>(sequence));
      }
    }
  }
};

bool bleReady() {
  return kEnableBleServer && gBleClientConnected && gBleStatusCharacteristic != nullptr &&
         gBleImageMetadataCharacteristic != nullptr && gBleImageDataCharacteristic != nullptr;
}

uint8_t bma400ReadRegister(uint8_t reg) {
  gBma400Spi.beginTransaction(SPISettings(kBma400SpiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(kBma400SpiCsPin, LOW);
  gBma400Spi.transfer(reg | 0x80);
  const uint8_t value = gBma400Spi.transfer(0x00);
  digitalWrite(kBma400SpiCsPin, HIGH);
  gBma400Spi.endTransaction();
  return value;
}

void bma400ReadRegisters(uint8_t startReg, uint8_t* data, size_t length) {
  if (data == nullptr || length == 0) {
    return;
  }
  gBma400Spi.beginTransaction(SPISettings(kBma400SpiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(kBma400SpiCsPin, LOW);
  gBma400Spi.transfer(startReg | 0x80);
  for (size_t i = 0; i < length; ++i) {
    data[i] = gBma400Spi.transfer(0x00);
  }
  digitalWrite(kBma400SpiCsPin, HIGH);
  gBma400Spi.endTransaction();
}

void bma400WriteRegister(uint8_t reg, uint8_t value) {
  gBma400Spi.beginTransaction(SPISettings(kBma400SpiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(kBma400SpiCsPin, LOW);
  gBma400Spi.transfer(reg & 0x7F);
  gBma400Spi.transfer(value);
  digitalWrite(kBma400SpiCsPin, HIGH);
  gBma400Spi.endTransaction();
}

int16_t decodeBma400Axis(uint8_t lsb, uint8_t msb) {
  int16_t value = static_cast<int16_t>(lsb | ((msb & 0x0F) << 8));
  if (value > 2047) {
    value -= 4096;
  }
  return value;
}

uint8_t bma400ReadRegisterWithMode(uint8_t reg, uint8_t spiMode) {
  gBma400Spi.beginTransaction(SPISettings(kBma400SpiHz, MSBFIRST, spiMode));
  digitalWrite(kBma400SpiCsPin, LOW);
  gBma400Spi.transfer(reg | 0x80);
  const uint8_t value = gBma400Spi.transfer(0x00);
  digitalWrite(kBma400SpiCsPin, HIGH);
  gBma400Spi.endTransaction();
  return value;
}

bool setupBma400() {
  pinMode(kBma400SpiCsPin, OUTPUT);
  digitalWrite(kBma400SpiCsPin, HIGH);
  gBma400Spi.begin(kBma400SpiSckPin, kBma400SpiMisoPin, kBma400SpiMosiPin, kBma400SpiCsPin);
  delayMicroseconds(10);

  const uint8_t mode0Dummy = bma400ReadRegisterWithMode(kBma400ChipIdReg, SPI_MODE0);
  const uint8_t mode0ChipId = bma400ReadRegisterWithMode(kBma400ChipIdReg, SPI_MODE0);
  const uint8_t mode3Dummy = bma400ReadRegisterWithMode(kBma400ChipIdReg, SPI_MODE3);
  const uint8_t mode3ChipId = bma400ReadRegisterWithMode(kBma400ChipIdReg, SPI_MODE3);
  Serial.printf(
      "BMA400 probe mode0 dummy=0x%02x chip_id=0x%02x | mode3 dummy=0x%02x chip_id=0x%02x\n",
      mode0Dummy, mode0ChipId, mode3Dummy, mode3ChipId);
  const uint8_t chipId =
      mode0ChipId == kBma400ExpectedChipId ? mode0ChipId
      : mode3ChipId == kBma400ExpectedChipId ? mode3ChipId
                                             : mode0ChipId;
  if (chipId != kBma400ExpectedChipId) {
    Serial.println("BMA400 probe failed");
    return false;
  }

  bma400WriteRegister(kBma400AccConfig0Reg, kBma400NormalMode);
  delayMicroseconds(1500);
  const uint8_t accConfig0 = bma400ReadRegister(kBma400AccConfig0Reg);
  Serial.printf("BMA400 normal mode set acc_config0=0x%02x\n", accConfig0);
  return true;
}

void logBma400MotionData(uint32_t nowMs) {
  if (!gBma400Ready || nowMs - gLastBma400LogMs < kBma400LogIntervalMs) {
    return;
  }
  gLastBma400LogMs = nowMs;

  uint8_t raw[6]{};
  bma400ReadRegisters(kBma400AccXlsbReg, raw, sizeof(raw));
  const int16_t x = decodeBma400Axis(raw[0], raw[1]);
  const int16_t y = decodeBma400Axis(raw[2], raw[3]);
  const int16_t z = decodeBma400Axis(raw[4], raw[5]);
  Serial.printf("BMA400 motion raw x=%d y=%d z=%d bytes=%02x %02x %02x %02x %02x %02x\n",
                x, y, z, raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
}

bool bleNotificationsEnabled(BLECharacteristic* characteristic) {
  if (characteristic == nullptr) {
    return false;
  }
  BLE2902* cccd = static_cast<BLE2902*>(characteristic->getDescriptorByUUID((uint16_t)0x2902));
  return cccd != nullptr && cccd->getNotifications();
}

uint8_t mapBleImageReasonCode(const char* reason) {
  if (reason == nullptr) {
    return 0;
  }
  if (strcmp(reason, "red_light_detected") == 0) {
    return 1;
  }
  if (strcmp(reason, "green_light_detected") == 0) {
    return 2;
  }
  if (strcmp(reason, "manual_request") == 0) {
    return 3;
  }
  return 0;
}

void clearBleActiveImage() {
  if (gBleActiveImageData != nullptr) {
    free(const_cast<uint8_t*>(gBleActiveImageData));
    gBleActiveImageData = nullptr;
  }
  gBleActiveImageSize = 0;
  gBleActiveImageWidth = 0;
  gBleActiveImageHeight = 0;
  gBleActiveImageReasonCode = 0;
  gBleActiveImageCapturedAtMs = 0;
  gBleActiveImageSequence = 0;
}

void clearInferenceBuffers() {
  if (gEiInputBuffer != nullptr) {
    free(gEiInputBuffer);
    gEiInputBuffer = nullptr;
    log_i("EI input buffer cleared");
  }
}

void logJpegEdgeBytes(const char* label, const uint8_t* data, size_t size) {
  if (data == nullptr || size == 0) {
    log_d("%s jpeg edge bytes unavailable size=%u",
          label != nullptr ? label : "BLE", static_cast<unsigned>(size));
    return;
  }
  char headBuffer[96];
  char tailBuffer[96];
  const size_t headCount = std::min<size_t>(8, size);
  const size_t tailCount = std::min<size_t>(8, size);
  int headOffset = snprintf(headBuffer, sizeof(headBuffer), "%s jpeg head:",
                            label != nullptr ? label : "BLE");
  for (size_t i = 0; i < headCount; ++i) {
    headOffset += snprintf(headBuffer + headOffset, sizeof(headBuffer) - headOffset, " %02X",
                           static_cast<unsigned>(data[i]));
  }
  int tailOffset = snprintf(tailBuffer, sizeof(tailBuffer), "%s jpeg tail:",
                            label != nullptr ? label : "BLE");
  for (size_t i = size - tailCount; i < size; ++i) {
    tailOffset += snprintf(tailBuffer + tailOffset, sizeof(tailBuffer) - tailOffset, " %02X",
                           static_cast<unsigned>(data[i]));
  }
  log_d("%s", headBuffer);
  log_d("%s", tailBuffer);
}

unsigned bleImageTotalChunks(size_t size) {
  return static_cast<unsigned>((size + kBleImageChunkPayloadSize - 1) / kBleImageChunkPayloadSize);
}

unsigned bleImageWindowChunkCount(size_t size) {
  const unsigned totalChunks = bleImageTotalChunks(size);
  return std::max(1U, static_cast<unsigned>((totalChunks + kBleImageWindowParts - 1) / kBleImageWindowParts));
}

void notifyBleText(BLECharacteristic* characteristic, const std::string& text) {
  if (!bleReady() || characteristic == nullptr) {
    log_d("BLE notify skipped ready=%d characteristic=%p", bleReady() ? 1 : 0,
          static_cast<void*>(characteristic));
    return;
  }
  const bool notificationsEnabled = bleNotificationsEnabled(characteristic);
  log_d("BLE notify attempt uuid=%s len=%u subscribed=%d peers=%u payload=%s",
        characteristic->getUUID().toString().c_str(), static_cast<unsigned>(text.size()),
        notificationsEnabled ? 1 : 0,
        static_cast<unsigned>(gBleServer != nullptr ? gBleServer->getPeerDevices(false).size() : 0),
        text.c_str());
  characteristic->setValue(reinterpret_cast<const uint8_t*>(text.data()), text.size());
  characteristic->notify();
}

void notifyBleBytes(BLECharacteristic* characteristic, const uint8_t* data, size_t size,
                    const char* label) {
  if (!bleReady() || characteristic == nullptr || data == nullptr || size == 0) {
    log_d("BLE notify skipped ready=%d characteristic=%p label=%s", bleReady() ? 1 : 0,
          static_cast<void*>(characteristic), label != nullptr ? label : "bytes");
    return;
  }
  const bool notificationsEnabled = bleNotificationsEnabled(characteristic);
  log_d("BLE notify attempt uuid=%s len=%u subscribed=%d peers=%u label=%s",
        characteristic->getUUID().toString().c_str(), static_cast<unsigned>(size),
        notificationsEnabled ? 1 : 0,
        static_cast<unsigned>(gBleServer != nullptr ? gBleServer->getPeerDevices(false).size() : 0),
        label != nullptr ? label : "bytes");
  characteristic->setValue(data, size);
  characteristic->notify();
}

void notifyBleImageChunks(const uint8_t* data, size_t size, uint16_t startSequence) {
  if (!bleReady() || gBleImageDataCharacteristic == nullptr || data == nullptr || size == 0) {
    log_d("BLE image chunks skipped ready=%d characteristic=%p data=%p size=%u",
          bleReady() ? 1 : 0, static_cast<void*>(gBleImageDataCharacteristic),
          static_cast<const void*>(data), static_cast<unsigned>(size));
    return;
  }

  const size_t startOffset = static_cast<size_t>(startSequence) * kBleImageChunkPayloadSize;
  if (startOffset >= size) {
    log_d("BLE image chunk resend skipped start_sequence=%u size=%u",
          static_cast<unsigned>(startSequence), static_cast<unsigned>(size));
    return;
  }

  size_t offset = startOffset;
  unsigned chunkIndex = startSequence;
  const unsigned totalChunks = bleImageTotalChunks(size);
  const unsigned windowChunkCount = bleImageWindowChunkCount(size);
  const unsigned endExclusive =
      std::min(totalChunks, static_cast<unsigned>(startSequence) + windowChunkCount);
  log_d("BLE image chunk window start total_bytes=%u chunk_payload=%u chunk_size=%u start_sequence=%u end_sequence=%u total_chunks=%u window_chunks=%u subscribed=%d peers=%u",
        static_cast<unsigned>(size), static_cast<unsigned>(kBleImageChunkPayloadSize),
        static_cast<unsigned>(kBleImageChunkSize), static_cast<unsigned>(startSequence),
        endExclusive > 0 ? endExclusive - 1 : 0, totalChunks, windowChunkCount,
        bleNotificationsEnabled(gBleImageDataCharacteristic) ? 1 : 0,
        static_cast<unsigned>(gBleServer != nullptr ? gBleServer->getPeerDevices(false).size() : 0));
  while (offset < size && chunkIndex < endExclusive) {
    const size_t chunkPayloadSize =
        std::min(static_cast<size_t>(kBleImageChunkPayloadSize), size - offset);
    ++chunkIndex;
    if (chunkIndex <= 3 || chunkIndex == totalChunks || (chunkIndex % 25) == 0) {
      log_d("BLE image chunk %u/%u offset=%u payload_len=%u", chunkIndex, totalChunks,
            static_cast<unsigned>(offset), static_cast<unsigned>(chunkPayloadSize));
    }
    uint8_t packet[kBleImageChunkSize];
    const uint16_t sequence = static_cast<uint16_t>(chunkIndex - 1);
    packet[0] = static_cast<uint8_t>(sequence & 0xFF);
    packet[1] = static_cast<uint8_t>((sequence >> 8) & 0xFF);
    memcpy(packet + kBleChunkSequenceHeaderSize, data + offset, chunkPayloadSize);
    gBleImageDataCharacteristic->setValue(packet, chunkPayloadSize + kBleChunkSequenceHeaderSize);
    gBleImageDataCharacteristic->notify();
    offset += chunkPayloadSize;
    delay(kBleChunkDelayMs);
  }
  log_d("BLE image chunk window complete start_sequence=%u next_sequence=%u total_chunks=%u",
        static_cast<unsigned>(startSequence), static_cast<unsigned>(chunkIndex), totalChunks);
}

std::string makeBleStatusJson(const char* state, const char* summary, bool redFound, bool greenOn,
                              uint32_t capturedAtMs) {
  (void)state;
  (void)summary;
  (void)capturedAtMs;
  return "{}";
}

std::string makeBleStatusJson(uint8_t statusCode, bool redFound, bool greenOn) {
  std::string json;
  json.reserve(24);
  json += "{\"s\":";
  json += std::to_string(static_cast<unsigned>(statusCode));
  json += ",\"r\":";
  json += redFound ? "1" : "0";
  json += ",\"g\":";
  json += greenOn ? "1" : "0";
  json += "}";
  return json;
}

void notifyBleStatus(const char* state, const char* summary, bool redFound, bool greenOn,
                     uint32_t nowMs, bool force = false) {
  if (!kEnableBleServer || gBleStatusCharacteristic == nullptr || !gBleClientConnected) {
    return;
  }
  (void)state;
  (void)summary;
  (void)nowMs;
  const std::string payload = makeBleStatusJson(gCurrentBleStatusCode, redFound, greenOn);
  if (!force && payload == gLastBleStatusPayload &&
      nowMs - gLastBleStatusSentAtMs < kBleStatusHeartbeatMs) {
    return;
  }
  gLastBleStatusPayload = payload;
  gLastBleStatusSentAtMs = nowMs;
  notifyBleText(gBleStatusCharacteristic, payload);
}

uint8_t mapBleStatusCode(const char* state) {
  if (strcmp(state, "Monitoring") == 0) return 0;
  if (strcmp(state, "Red detected") == 0) return 1;
  if (strcmp(state, "Watching green") == 0) return 2;
  if (strcmp(state, "Green observed") == 0) return 3;
  if (strcmp(state, "Scene changed") == 0) return 4;
  if (strcmp(state, "Sending image") == 0) return 5;
  if (strcmp(state, "Paused") == 0) return 6;
  return 7;
}

void setCurrentBleStatus(const char* state, const char* summary, bool redFound, bool greenOn,
                         uint32_t nowMs, bool forceNotify) {
  gCurrentBleStatusCode = mapBleStatusCode(state);
  gCurrentBleStatusRedFound = redFound;
  gCurrentBleStatusGreenOn = greenOn;
  gHasCurrentBleStatus = true;
  notifyBleStatus(state, summary, redFound, greenOn, nowMs, forceNotify);
}

void notifyCurrentBleStatusOnLoop(uint32_t nowMs) {
  if (!gHasCurrentBleStatus) {
    return;
  }
  notifyBleStatus("", "", gCurrentBleStatusRedFound, gCurrentBleStatusGreenOn, nowMs, false);
}

void sendBleJpegImage(const uint8_t* jpegData, size_t jpegSize, uint16_t width, uint16_t height,
                      const char* reason, uint32_t capturedAtMs) {
  if (!bleReady() || jpegData == nullptr || jpegSize == 0) {
    log_d("BLE image send skipped ready=%d data=%p size=%u reason=%s",
          bleReady() ? 1 : 0, static_cast<const void*>(jpegData),
          static_cast<unsigned>(jpegSize), reason != nullptr ? reason : "<null>");
    return;
  }

  uint8_t metadata[15];
  const uint8_t reasonCode = mapBleImageReasonCode(reason);
  metadata[0] = reasonCode;
  metadata[1] = static_cast<uint8_t>(width & 0xFF);
  metadata[2] = static_cast<uint8_t>((width >> 8) & 0xFF);
  metadata[3] = static_cast<uint8_t>(height & 0xFF);
  metadata[4] = static_cast<uint8_t>((height >> 8) & 0xFF);
  metadata[5] = static_cast<uint8_t>(jpegSize & 0xFF);
  metadata[6] = static_cast<uint8_t>((jpegSize >> 8) & 0xFF);
  metadata[7] = static_cast<uint8_t>((jpegSize >> 16) & 0xFF);
  metadata[8] = static_cast<uint8_t>((jpegSize >> 24) & 0xFF);
  metadata[9] = static_cast<uint8_t>(capturedAtMs & 0xFF);
  metadata[10] = static_cast<uint8_t>((capturedAtMs >> 8) & 0xFF);
  metadata[11] = static_cast<uint8_t>((capturedAtMs >> 16) & 0xFF);
  metadata[12] = static_cast<uint8_t>((capturedAtMs >> 24) & 0xFF);
  metadata[13] = static_cast<uint8_t>(kBleImageChunkPayloadSize & 0xFF);
  metadata[14] = static_cast<uint8_t>((kBleImageChunkPayloadSize >> 8) & 0xFF);

  clearBleActiveImage();
  gBleActiveImageData = jpegData;
  gBleActiveImageSize = jpegSize;
  gBleActiveImageWidth = width;
  gBleActiveImageHeight = height;
  gBleActiveImageReasonCode = reasonCode;
  gBleActiveImageCapturedAtMs = capturedAtMs;
  gBleActiveImageSequence = ++gPacketSequence;
  log_i("BLE image send start reason=%s reason_code=%u width=%u height=%u jpeg_bytes=%u captured_at=%lu seq=%lu chunk_payload=%u metadata_lead_delay=%lu",
        reason != nullptr ? reason : "<null>", static_cast<unsigned>(reasonCode),
        static_cast<unsigned>(width), static_cast<unsigned>(height),
        static_cast<unsigned>(jpegSize), static_cast<unsigned long>(capturedAtMs),
        static_cast<unsigned long>(gBleActiveImageSequence),
        static_cast<unsigned>(kBleImageChunkPayloadSize),
        static_cast<unsigned long>(kBleMetadataLeadDelayMs));
  notifyBleBytes(gBleImageMetadataCharacteristic, metadata, sizeof(metadata), "image_metadata");
  delay(kBleMetadataLeadDelayMs);
  notifyBleImageChunks(gBleActiveImageData, gBleActiveImageSize, 0);
  log_i("BLE image send complete reason=%s seq=%lu",
        reason != nullptr ? reason : "<null>",
        static_cast<unsigned long>(gBleActiveImageSequence));
}

void sendBleFrameSnapshot(const camera_fb_t* fb, const char* reason, uint32_t capturedAtMs) {
  if (!sendImagesBle) {
    log_d("BLE image sending disabled");
    return;
  }
  if (!bleReady() || fb == nullptr) {
    log_d("BLE frame snapshot skipped ready=%d fb=%p reason=%s", bleReady() ? 1 : 0,
          static_cast<const void*>(fb), reason != nullptr ? reason : "<null>");
    return;
  }

  log_d("BLE frame snapshot begin reason=%s frame=%ux%u len=%u captured_at=%lu",
        reason != nullptr ? reason : "<null>", static_cast<unsigned>(fb->width),
        static_cast<unsigned>(fb->height), static_cast<unsigned>(fb->len),
        static_cast<unsigned long>(capturedAtMs));

  uint8_t* jpegData = nullptr;
  size_t jpegSize = 0;
  uint16_t bleImageWidth = fb->width;
  uint16_t bleImageHeight = fb->height;
  uint8_t* bleJpegRgb888Buffer = nullptr;

  if (kBleImageSource == BleImageSource::EdgeImpulseInput) {
    EdgeImpulseTransform transform;
    if (!prepareEdgeImpulseInput(fb, transform)) {
      log_e("BLE snapshot EI input prepare failed reason=%s",
            reason != nullptr ? reason : "<null>");
      return;
    }
    bleImageWidth = EI_CLASSIFIER_INPUT_WIDTH;
    bleImageHeight = EI_CLASSIFIER_INPUT_HEIGHT;
    bleJpegRgb888Buffer = createBleJpegRgb888BufferFromEiInput(
        static_cast<size_t>(bleImageWidth) * static_cast<size_t>(bleImageHeight));
    if (bleJpegRgb888Buffer == nullptr) {
      log_e("BLE snapshot EI channel-swap alloc failed reason=%s",
            reason != nullptr ? reason : "<null>");
      return;
    }
    if (!fmt2jpg(bleJpegRgb888Buffer,
                 bleImageWidth * bleImageHeight * 3,
                 bleImageWidth,
                 bleImageHeight,
                 PIXFORMAT_RGB888,
                 kBleSnapshotJpegQuality,
                 &jpegData,
                 &jpegSize) ||
        jpegData == nullptr || jpegSize == 0) {
      log_e("BLE snapshot EI encode failed reason=%s",
            reason != nullptr ? reason : "<null>");
      free(bleJpegRgb888Buffer);
      return;
    }
  } else {
    if (!fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, kBleSnapshotJpegQuality,
                 &jpegData, &jpegSize) ||
        jpegData == nullptr || jpegSize == 0) {
      log_e("BLE snapshot encode failed reason=%s",
            reason != nullptr ? reason : "<null>");
      return;
    }
  }

  log_i("BLE snapshot encoded reason=%s source=%s jpeg_bytes=%u quality=%u size=%ux%u",
        reason != nullptr ? reason : "<null>",
        kBleImageSource == BleImageSource::EdgeImpulseInput ? "ei_160" : "full_frame",
        static_cast<unsigned>(jpegSize),
        static_cast<unsigned>(kBleSnapshotJpegQuality),
        static_cast<unsigned>(bleImageWidth), static_cast<unsigned>(bleImageHeight));
  logJpegEdgeBytes("BLE tx", jpegData, jpegSize);

  sendBleJpegImage(jpegData, jpegSize, bleImageWidth, bleImageHeight, reason, capturedAtMs);
  if (bleJpegRgb888Buffer != nullptr) {
    free(bleJpegRgb888Buffer);
  }
  log_d("BLE frame snapshot end reason=%s", reason != nullptr ? reason : "<null>");
}

uint8_t* createBleJpegRgb888BufferFromEiInput(size_t pixelCount) {
  if (gEiInputBuffer == nullptr || pixelCount == 0) {
    return nullptr;
  }
  uint8_t* buffer = static_cast<uint8_t*>(malloc(pixelCount * 3));
  if (buffer == nullptr) {
    return nullptr;
  }
  for (size_t i = 0; i < pixelCount; ++i) {
    const size_t offset = i * 3;
    buffer[offset + 0] = gEiInputBuffer[offset + 2];
    buffer[offset + 1] = gEiInputBuffer[offset + 1];
    buffer[offset + 2] = gEiInputBuffer[offset + 0];
  }
  return buffer;
}

void setupBleServer() {
  if (!kEnableBleServer) {
    return;
  }

  BLEDevice::init(kBleDeviceName);
  BLEDevice::setMTU(kBlePreferredMtu);
  gBleServer = BLEDevice::createServer();
  gBleServer->setCallbacks(new TrafficCamBleServerCallbacks());

  BLEService* service = gBleServer->createService(kBleServiceUuid);
  gBleStatusCharacteristic = service->createCharacteristic(
      kBleStatusCharacteristicUuid, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  gBleImageMetadataCharacteristic = service->createCharacteristic(
      kBleImageMetadataCharacteristicUuid, BLECharacteristic::PROPERTY_NOTIFY);
  gBleImageDataCharacteristic =
      service->createCharacteristic(kBleImageDataCharacteristicUuid, BLECharacteristic::PROPERTY_NOTIFY);
  gBleControlCharacteristic = service->createCharacteristic(
      kBleControlCharacteristicUuid, BLECharacteristic::PROPERTY_WRITE);

  gBleStatusCharacteristic->setCallbacks(new TrafficCamBleStatusCallbacks());
  gBleControlCharacteristic->setCallbacks(new TrafficCamBleControlCallbacks());

  service->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(kBleServiceUuid);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  log_i("BLE advertising started name=%s", kBleDeviceName);
  log_i("BLE service uuid=%s", kBleServiceUuid);
  log_i("BLE preferred mtu=%u image_chunk_payload=%u image_chunk_size=%u ble_jpeg_quality=%u ble_image_source=%s",
        static_cast<unsigned>(kBlePreferredMtu),
        static_cast<unsigned>(kBleImageChunkPayloadSize),
        static_cast<unsigned>(kBleImageChunkSize),
        static_cast<unsigned>(kBleSnapshotJpegQuality),
        kBleImageSource == BleImageSource::EdgeImpulseInput ? "ei_160" : "full_frame");
}

void unpackRgb565(const uint8_t* data, int index, int& r, int& g, int& b) {
  const uint16_t pixel = (static_cast<uint16_t>(data[index]) << 8) | data[index + 1];
  r = ((pixel >> 11) & 0x1F) * 255 / 31;
  g = ((pixel >> 5) & 0x3F) * 255 / 63;
  b = (pixel & 0x1F) * 255 / 31;
}

void ei_printf(const char* format, ...) {
  va_list args;
  va_start(args, format);
  char buffer[256];
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.print(buffer);
}

int computeLuma(int r, int g, int b) {
  return (77 * r + 150 * g + 29 * b) >> 8;
}

bool isRedPixel(int r, int g, int b) {
  return r >= kMinRedComponent && (r - g) >= kMinRedExcessOverGreen &&
         (r - b) >= kMinRedExcessOverBlue;
}

bool isWarmBrightPixel(int r, int g, int b) {
  return r >= kEiWarmBrightMinRedComponent &&
         g >= kEiWarmBrightMinGreenComponent &&
         r >= g &&
         g >= b &&
         b <= kEiWarmBrightMaxBlueComponent &&
         (g - b) >= kEiWarmBrightMinBlueGap;
}

bool isDarkPixel(int r, int g, int b) {
  return computeLuma(r, g, b) <= kGreenWatchDarkLumaThreshold;
}

using RedClusterCheckResult = redbox::RedClusterCheckResult;

RedClusterCheckResult evaluateRedClusterInBox(const camera_fb_t* fb, int minX, int minY, int maxX,
                                              int maxY) {
  if (fb == nullptr) {
    return {};
  }
  return redbox::evaluateRedClusterInBox(fb->buf, static_cast<int>(fb->width),
                                         static_cast<int>(fb->height), minX, minY, maxX, maxY);
}

int scaleCoordinate(int value, int sourceExtent, int targetExtent) {
  if (sourceExtent <= 1 || targetExtent <= 1) {
    return 0;
  }
  return std::clamp((value * targetExtent) / sourceExtent, 0, targetExtent - 1);
}

traffic::LampDetection scaleLampDetection(const traffic::LampDetection& red, int sourceWidth,
                                          int sourceHeight, int targetWidth, int targetHeight) {
  traffic::LampDetection scaled = red;
  scaled.min_x = scaleCoordinate(red.min_x, sourceWidth, targetWidth);
  scaled.min_y = scaleCoordinate(red.min_y, sourceHeight, targetHeight);
  scaled.max_x = std::max(scaled.min_x, scaleCoordinate(red.max_x, sourceWidth, targetWidth));
  scaled.max_y = std::max(scaled.min_y, scaleCoordinate(red.max_y, sourceHeight, targetHeight));
  scaled.center_x =
      std::clamp(scaleCoordinate(red.center_x, sourceWidth, targetWidth), scaled.min_x, scaled.max_x);
  scaled.center_y = std::clamp(scaleCoordinate(red.center_y, sourceHeight, targetHeight),
                               scaled.min_y, scaled.max_y);
  scaled.radius = std::max(2, std::max(scaled.max_x - scaled.min_x, scaled.max_y - scaled.min_y) / 2);
  return scaled;
}

void padBoundingBox(int& x1, int& y1, int& x2, int& y2, int frameWidth, int frameHeight) {
  const int width = std::max(1, x2 - x1 + 1);
  const int height = std::max(1, y2 - y1 + 1);
  const int padX = std::max(1, static_cast<int>(std::ceil(width * kEiBoundingBoxPaddingFraction)));
  const int padY = std::max(1, static_cast<int>(std::ceil(height * kEiBoundingBoxPaddingFraction)));
  x1 = std::max(0, x1 - padX);
  y1 = std::max(0, y1 - padY);
  x2 = std::min(frameWidth - 1, x2 + padX);
  y2 = std::min(frameHeight - 1, y2 + padY);
}

bool clusterTouchesBoxEdge(const RedClusterCheckResult& result, int boxWidth, int boxHeight) {
  if (!result.accepted || boxWidth <= 0 || boxHeight <= 0) {
    return false;
  }
  return result.clusterMinX <= 0 || result.clusterMinY <= 0 ||
         result.clusterMaxX >= boxWidth - 1 || result.clusterMaxY >= boxHeight - 1;
}

void expandBoundingBox(int& x1, int& y1, int& x2, int& y2, int frameWidth, int frameHeight) {
  const int width = std::max(1, x2 - x1 + 1);
  const int height = std::max(1, y2 - y1 + 1);
  const int expandX =
      std::max(1, static_cast<int>(std::ceil(width * kEiClusterEdgeExpansionFraction)));
  const int expandY =
      std::max(1, static_cast<int>(std::ceil(height * kEiClusterEdgeExpansionFraction)));
  x1 = std::max(0, x1 - expandX);
  y1 = std::max(0, y1 - expandY);
  x2 = std::min(frameWidth - 1, x2 + expandX);
  y2 = std::min(frameHeight - 1, y2 + expandY);
}

bool detectRedStillOnInRegion(const camera_fb_t* fb, int centerX, int centerY, int radius,
                              int& redPixelCount) {
  const int width = static_cast<int>(fb->width);
  const int height = static_cast<int>(fb->height);
  const uint8_t* data = fb->buf;
  const int minX = max(0, centerX - radius);
  const int maxX = min(width - 1, centerX + radius);
  const int minY = max(0, centerY - radius);
  const int maxY = min(height - 1, centerY + radius);
  const int radiusSquared = radius * radius;

  redPixelCount = 0;

  for (int y = minY; y <= maxY; y += kScanStride) {
    for (int x = minX; x <= maxX; x += kScanStride) {
      const int dx = x - centerX;
      const int dy = y - centerY;
      if (dx * dx + dy * dy > radiusSquared) {
        continue;
      }

      const int index = (y * width + x) * 2;
      int r = 0;
      int g = 0;
      int b = 0;
      unpackRgb565(data, index, r, g, b);
      if (isRedPixel(r, g, b)) {
        ++redPixelCount;
      }
    }
  }

  return redPixelCount >= max(6, radius);
}

bool findSimilarRedPixelNearCenter(const camera_fb_t* fb, const GreenWatchState& watch,
                                   int originX, int originY, int& foundX, int& foundY, int& foundR,
                                   int& foundG, int& foundB, int& foundLuma) {
  const int width = static_cast<int>(fb->width);
  const int height = static_cast<int>(fb->height);
  const int halfBox = kGreenWatchCenterSearchBoxSize / 2;
  const int minX = std::max(0, originX - halfBox);
  const int maxX = std::min(width - 1, minX + kGreenWatchCenterSearchBoxSize - 1);
  const int minY = std::max(0, originY - halfBox);
  const int maxY = std::min(height - 1, minY + kGreenWatchCenterSearchBoxSize - 1);

  bool found = false;
  int bestScore = INT_MAX;

  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      int r = 0;
      int g = 0;
      int b = 0;
      unpackRgb565(fb->buf, (y * width + x) * 2, r, g, b);
      if (!isRedPixel(r, g, b)) {
        continue;
      }

      const int redDelta = std::abs(r - watch.baselineCenterR);
      const int greenDelta = std::abs(g - watch.baselineCenterG);
      const int blueDelta = std::abs(b - watch.baselineCenterB);
      if (redDelta > kGreenWatchSimilarRedDelta ||
          greenDelta > kGreenWatchSimilarGreenDelta ||
          blueDelta > kGreenWatchSimilarBlueDelta) {
        continue;
      }

      const int distancePenalty = std::abs(x - originX) + std::abs(y - originY);
      const int colorPenalty = redDelta + greenDelta + blueDelta;
      const int score = colorPenalty * 4 + distancePenalty;
      if (!found || score < bestScore) {
        found = true;
        bestScore = score;
        foundX = x;
        foundY = y;
        foundR = r;
        foundG = g;
        foundB = b;
        foundLuma = computeLuma(r, g, b);
      }
    }
  }

  return found;
}

void buildPatchSignatureScaled(const camera_fb_t* fb, int targetWidth, int targetHeight, int patchX,
                               int patchY, int patchWidth, int patchHeight, uint8_t* outSignature) {
  const int sourceWidth = static_cast<int>(fb->width);
  const int sourceHeight = static_cast<int>(fb->height);
  for (int gy = 0; gy < kPatchSignatureSize; ++gy) {
    for (int gx = 0; gx < kPatchSignatureSize; ++gx) {
      const int targetSampleX = std::clamp(
          patchX + (gx * std::max(1, patchWidth - 1)) / std::max(1, kPatchSignatureSize - 1), 0,
          targetWidth - 1);
      const int targetSampleY = std::clamp(
          patchY + (gy * std::max(1, patchHeight - 1)) / std::max(1, kPatchSignatureSize - 1), 0,
          targetHeight - 1);
      const int sampleX = scaleCoordinate(targetSampleX, targetWidth, sourceWidth);
      const int sampleY = scaleCoordinate(targetSampleY, targetHeight, sourceHeight);
      int r = 0;
      int g = 0;
      int b = 0;
      unpackRgb565(fb->buf, (sampleY * sourceWidth + sampleX) * 2, r, g, b);
      outSignature[gy * kPatchSignatureSize + gx] = static_cast<uint8_t>(computeLuma(r, g, b));
    }
  }
}

float computeSceneDifference(const camera_fb_t* fb, const GreenWatchState& watch) {
  uint8_t currentPatch[kPatchSignatureSize * kPatchSignatureSize]{};
  buildPatchSignatureScaled(fb, watch.frameWidth, watch.frameHeight, watch.patchX, watch.patchY,
                            watch.patchWidth, watch.patchHeight, currentPatch);
  int totalDiff = 0;
  for (int i = 0; i < kPatchSignatureSize * kPatchSignatureSize; ++i) {
    totalDiff += std::abs(static_cast<int>(currentPatch[i]) - static_cast<int>(watch.patchSignature[i]));
  }
  return static_cast<float>(totalDiff) /
         static_cast<float>(kPatchSignatureSize * kPatchSignatureSize);
}

void refreshGreenWatchPatchSignature(const camera_fb_t* fb, GreenWatchState& watch) {
  buildPatchSignatureScaled(fb, watch.frameWidth, watch.frameHeight, watch.patchX, watch.patchY,
                            watch.patchWidth, watch.patchHeight, watch.patchSignature);
}

void stopGreenWatch(const char* reason) {
  if (gGreenWatch.active) {
    log_i("GREEN_WATCH stop reason=%s", reason);
  }
  gGreenWatch = GreenWatchState{};
}

void startGreenWatch(const camera_fb_t* fb, const traffic::LampDetection& red, uint32_t nowMs) {
  GreenWatchState watch{};
  watch.active = true;
  watch.startedAtMs = nowMs;
  watch.lastCheckAtMs = nowMs;
  watch.sceneCheckReadyAtMs = nowMs + kGreenWatchSceneChangeGraceMs;
  watch.frameWidth = static_cast<int>(fb->width);
  watch.frameHeight = static_cast<int>(fb->height);

  const int sourceWidth = watch.frameWidth;
  const int sourceHeight = watch.frameHeight;
  const int minX = std::clamp(red.min_x, 0, watch.frameWidth - 1);
  const int minY = std::clamp(red.min_y, 0, watch.frameHeight - 1);
  const int maxX = std::clamp(red.max_x, minX, watch.frameWidth - 1);
  const int maxY = std::clamp(red.max_y, minY, watch.frameHeight - 1);
  watch.red = red;
  watch.red.min_x = minX;
  watch.red.min_y = minY;
  watch.red.max_x = maxX;
  watch.red.max_y = maxY;
  watch.red.center_x = std::clamp(red.center_x, minX, maxX);
  watch.red.center_y = std::clamp(red.center_y, minY, maxY);
  watch.red.radius = std::max(2, std::max(maxX - minX, maxY - minY) / 2);
  int centerR = 0;
  int centerG = 0;
  int centerB = 0;
  unpackRgb565(fb->buf, (watch.red.center_y * sourceWidth + watch.red.center_x) * 2,
               centerR, centerG, centerB);
  watch.baselineCenterR = centerR;
  watch.baselineCenterG = centerG;
  watch.baselineCenterB = centerB;
  watch.baselineCenterLuma = computeLuma(centerR, centerG, centerB);

  const int boxWidth = std::max(1, maxX - minX + 1);
  const int boxHeight = std::max(1, maxY - minY + 1);
  const int coreWidth = std::max(3, (boxWidth + 1) / 2);
  const int coreHeight = std::max(3, (boxHeight + 1) / 2);
  const int coreX = std::clamp(watch.red.center_x - coreWidth / 2, minX, maxX);
  const int coreY = std::clamp(watch.red.center_y - coreHeight / 2, minY, maxY);
  const int coreMaxX = std::min(maxX, coreX + coreWidth - 1);
  const int coreMaxY = std::min(maxY, coreY + coreHeight - 1);
  watch.coreX = coreX;
  watch.coreY = coreY;
  watch.coreWidth = coreMaxX - coreX + 1;
  watch.coreHeight = coreMaxY - coreY + 1;

  for (int y = coreY; y <= coreMaxY && watch.trackedPixelCount < kMaxTrackedRedPixels; ++y) {
    for (int x = coreX; x <= coreMaxX && watch.trackedPixelCount < kMaxTrackedRedPixels; ++x) {
      int r = 0;
      int g = 0;
      int b = 0;
      unpackRgb565(fb->buf, (y * sourceWidth + x) * 2, r, g, b);
      if (!isRedPixel(r, g, b)) {
        continue;
      }
      watch.trackedPixels[watch.trackedPixelCount++] = {
          static_cast<uint16_t>(x), static_cast<uint16_t>(y)};
    }
  }

  if (watch.trackedPixelCount < static_cast<size_t>(kMinRedPixelsForDetection)) {
    for (int y = coreY; y <= coreMaxY && watch.trackedPixelCount < kMaxTrackedRedPixels; ++y) {
      for (int x = coreX; x <= coreMaxX && watch.trackedPixelCount < kMaxTrackedRedPixels; ++x) {
        watch.trackedPixels[watch.trackedPixelCount++] = {
            static_cast<uint16_t>(x), static_cast<uint16_t>(y)};
      }
    }
  }
  const int patchWidth =
      std::max(boxWidth, static_cast<int>(std::ceil(boxWidth * kGreenWatchPatchScale)));
  const int patchHeight =
      std::max(boxHeight, static_cast<int>(std::ceil(boxHeight * kGreenWatchPatchScale)));
  watch.patchX =
      std::clamp(watch.red.center_x - patchWidth / 2, 0, std::max(0, watch.frameWidth - patchWidth));
  watch.patchY = std::clamp(watch.red.center_y - patchHeight / 2, 0,
                            std::max(0, watch.frameHeight - patchHeight));
  watch.patchWidth = std::min(watch.frameWidth - watch.patchX, patchWidth);
  watch.patchHeight = std::min(watch.frameHeight - watch.patchY, patchHeight);
  buildPatchSignatureScaled(fb, watch.frameWidth, watch.frameHeight, watch.patchX, watch.patchY,
                            watch.patchWidth, watch.patchHeight, watch.patchSignature);

  gGreenWatch = watch;
  log_d("GREEN_WATCH start center=(%d,%d) tracked_pixels=%u tracking_frame=%dx%d red_box=(%d,%d,%d,%d) core_box=(%d,%d,%d,%d) patch=(%d,%d,%d,%d)",
        watch.red.center_x, watch.red.center_y, static_cast<unsigned>(watch.trackedPixelCount),
        watch.frameWidth, watch.frameHeight,
        minX, minY, maxX, maxY,
        watch.coreX, watch.coreY,
        watch.coreX + watch.coreWidth - 1, watch.coreY + watch.coreHeight - 1,
        watch.patchX, watch.patchY, watch.patchWidth, watch.patchHeight);
}

bool enterGreenWatchLowResolutionMode(uint32_t nowMs) {
  (void)nowMs;
  return true;
}

bool exitGreenWatchLowResolutionMode() {
  return true;
}

GreenWatchUpdateResult updateGreenWatch(const camera_fb_t* fb, uint32_t nowMs) {
  if (!gGreenWatch.active) {
    return GreenWatchUpdateResult::kNone;
  }
  if (nowMs - gGreenWatch.startedAtMs >= kGreenWatchTimeoutMs) {
    stopGreenWatch("timeout");
    return GreenWatchUpdateResult::kNone;
  }
  if (nowMs - gGreenWatch.lastCheckAtMs < kGreenWatchCheckIntervalMs) {
    return GreenWatchUpdateResult::kNone;
  }
  gGreenWatch.lastCheckAtMs = nowMs;

  const int width = static_cast<int>(fb->width);
  const int height = static_cast<int>(fb->height);
  const int originX = std::clamp(gGreenWatch.red.center_x, 0, width - 1);
  const int originY = std::clamp(gGreenWatch.red.center_y, 0, height - 1);
  int centerX = originX;
  int centerY = originY;
  int centerR = 0;
  int centerG = 0;
  int centerB = 0;
  int centerLuma = 0;
  const bool foundSimilarPixel =
      findSimilarRedPixelNearCenter(fb, gGreenWatch, originX, originY,
                                    centerX, centerY, centerR, centerG, centerB, centerLuma);
  const bool centerDark = centerLuma <= kGreenWatchCenterDarkLumaThreshold;
  const bool centerDropped =
      centerLuma <= std::max(0, gGreenWatch.baselineCenterLuma - kGreenWatchCenterDarkDelta);
  const float sceneDiff = computeSceneDifference(fb, gGreenWatch);
  log_d("GREEN_WATCH check diff=%.2f found_similar=%d origin=(%d,%d) center=(%d,%d) rgb=(%d,%d,%d) luma=%d baseline_rgb=(%d,%d,%d) baseline_luma=%d center_dark=%d center_dropped=%d core_box=(%d,%d,%d,%d)",
        sceneDiff,
        foundSimilarPixel ? 1 : 0, originX, originY,
        centerX, centerY, centerR, centerG, centerB, centerLuma,
        gGreenWatch.baselineCenterR, gGreenWatch.baselineCenterG, gGreenWatch.baselineCenterB,
        gGreenWatch.baselineCenterLuma, centerDark ? 1 : 0, centerDropped ? 1 : 0,
        gGreenWatch.coreX, gGreenWatch.coreY,
        gGreenWatch.coreX + gGreenWatch.coreWidth - 1,
        gGreenWatch.coreY + gGreenWatch.coreHeight - 1);

  if (foundSimilarPixel) {
    if (nowMs < gGreenWatch.sceneCheckReadyAtMs) {
      refreshGreenWatchPatchSignature(fb, gGreenWatch);
    } else if (sceneDiff > kGreenWatchSceneDiffThreshold) {
      log_i("GREEN_WATCH scene_changed diff=%.2f", sceneDiff);
      stopGreenWatch("scene_changed");
      return GreenWatchUpdateResult::kSceneChanged;
    }
    return GreenWatchUpdateResult::kNone;
  }

  if (nowMs < gGreenWatch.sceneCheckReadyAtMs) {
    refreshGreenWatchPatchSignature(fb, gGreenWatch);
  } else if (sceneDiff > kGreenWatchSceneDiffThreshold) {
    log_i("GREEN_WATCH scene_changed diff=%.2f", sceneDiff);
    stopGreenWatch("scene_changed");
    return GreenWatchUpdateResult::kSceneChanged;
  }

  log_i("GREEN_WATCH red_off found_similar=%d center=(%d,%d) rgb=(%d,%d,%d) luma=%d baseline_luma=%d center_dark=%d center_dropped=%d",
        foundSimilarPixel ? 1 : 0,
        centerX, centerY, centerR, centerG, centerB, centerLuma,
        gGreenWatch.baselineCenterLuma, centerDark ? 1 : 0, centerDropped ? 1 : 0);
  stopGreenWatch("red_off");
  return GreenWatchUpdateResult::kRedOffDetected;

}

float measureBrightPixelFraction(const camera_fb_t* fb) {
  const int width = static_cast<int>(fb->width);
  const int height = static_cast<int>(fb->height);
  int brightCount = 0;
  int sampleCount = 0;

  for (int y = 0; y < height; y += kSunSampleStride) {
    for (int x = 0; x < width; x += kSunSampleStride) {
      int r = 0;
      int g = 0;
      int b = 0;
      unpackRgb565(fb->buf, (y * width + x) * 2, r, g, b);
      if (computeLuma(r, g, b) >= kSunBrightLumaThreshold) {
        ++brightCount;
      }
      ++sampleCount;
    }
  }

  if (sampleCount == 0) {
    return 0.0f;
  }
  return static_cast<float>(brightCount) / static_cast<float>(sampleCount);
}

void adjustExposureForSun(const camera_fb_t* fb) {
  if (!kEnableSunExposureCompensation) {
    return;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    return;
  }

  const float brightFraction = measureBrightPixelFraction(fb);
  int targetAeLevel = gCurrentAeLevel;
  if (brightFraction > kSunBrightFractionThreshold) {
    targetAeLevel = kSunAeLevel;
  } else if (brightFraction < kSunRecoverFractionThreshold) {
    targetAeLevel = kAeLevel;
  }

  if (targetAeLevel != gCurrentAeLevel) {
    sensor->set_ae_level(sensor, targetAeLevel);
    gCurrentAeLevel = targetAeLevel;
    log_d("EXPOSURE bright_fraction=%.3f ae_level=%d", brightFraction, gCurrentAeLevel);
  }
}

void blinkFatal() {
  for (;;) {
    delay(1000);
  }
}

camera_fb_t* captureFrameLocked() {
  if (gCameraMutex == nullptr) {
    log_e("CAPTURE mutex missing");
    return nullptr;
  }
  if (xSemaphoreTake(gCameraMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    log_e("CAPTURE mutex timeout");
    return nullptr;
  }
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb == nullptr) {
    ++gCaptureFailureCount;
    if (gCaptureFailureCount <= 10 || (gCaptureFailureCount % 50) == 0) {
      Serial.printf("CAPTURE fb_get failed count=%lu\n",
                    static_cast<unsigned long>(gCaptureFailureCount));
    }
    xSemaphoreGive(gCameraMutex);
  } else {
    ++gFrameCounter;
    if (gFrameCounter <= 5 || (gFrameCounter % 20) == 0) {
      Serial.printf("CAPTURE fb_get ok count=%lu size=%ux%u len=%u\n",
                    static_cast<unsigned long>(gFrameCounter), fb->width, fb->height,
                    static_cast<unsigned>(fb->len));
    }
  }
  return fb;
}

void releaseFrameLocked(camera_fb_t* fb) {
  if (fb != nullptr) {
    esp_camera_fb_return(fb);
  }
  if (gCameraMutex != nullptr) {
    xSemaphoreGive(gCameraMutex);
  }
}

framesize_t defaultCameraFrameSize() {
  return gHasPsram ? kFrameSizeWithPsram : kFrameSizeWithoutPsram;
}

bool frameSizeToResolution(framesize_t frameSize, int& width, int& height) {
  switch (frameSize) {
    case FRAMESIZE_QQVGA: width = 160; height = 120; return true;
    case FRAMESIZE_QVGA: width = 320; height = 240; return true;
    case FRAMESIZE_VGA: width = 640; height = 480; return true;
    case FRAMESIZE_SVGA: width = 800; height = 600; return true;
    case FRAMESIZE_XGA: width = 1024; height = 768; return true;
    case FRAMESIZE_HD: width = 1280; height = 720; return true;
    case FRAMESIZE_SXGA: width = 1280; height = 1024; return true;
    case FRAMESIZE_UXGA: width = 1600; height = 1200; return true;
    case FRAMESIZE_QXGA: width = 2048; height = 1536; return true;
    case FRAMESIZE_5MP: width = 2592; height = 1944; return true;
    default: return false;
  }
}

void applyCameraSensorSettings(sensor_t* sensor) {
  if (sensor == nullptr) {
    log_e("CAMERA sensor unavailable");
    return;
  }

  sensor->set_hmirror(sensor, kSensorHorizontalMirror ? 1 : 0);
  sensor->set_vflip(sensor, kSensorVerticalFlip ? 1 : 0);
  sensor->set_ae_level(sensor, kAeLevel);
  gCurrentAeLevel = kAeLevel;
  if (kDumpOv5640PresetRegisters) {
    dumpOv5640PresetRegisters(sensor, "before_window");
  }
  applyOv5640RawWindow(sensor);
  applyOv5640SensorDigitalZoom(sensor);
  log_i("CAMERA sensor configured hmirror=%d vflip=%d ae_level=%d",
        kSensorHorizontalMirror ? 1 : 0, kSensorVerticalFlip ? 1 : 0, gCurrentAeLevel);
}

bool reinitializeCameraForFrameSize(framesize_t frameSize, const char* reason) {
  if (gCameraMutex == nullptr) {
    log_e("CAMERA reinit failed mutex missing");
    return false;
  }
  if (xSemaphoreTake(gCameraMutex, pdMS_TO_TICKS(3000)) != pdTRUE) {
    log_e("CAMERA reinit failed mutex timeout");
    return false;
  }

  log_i("CAMERA reinit begin reason=%s frame_size=%d", reason != nullptr ? reason : "unknown",
        static_cast<int>(frameSize));
  if (gCameraInitialized) {
    clearBleActiveImage();
    clearInferenceBuffers();
    esp_camera_deinit();
    gCameraInitialized = false;
    delay(75);
  }

  camera_config_t config = makeCameraConfig();
  config.frame_size = frameSize;
  const esp_err_t initResult = esp_camera_init(&config);
  if (initResult != ESP_OK) {
    log_e("CAMERA reinit failed reason=%s frame_size=%d err=0x%x",
          reason != nullptr ? reason : "unknown", static_cast<int>(frameSize),
          static_cast<unsigned>(initResult));
    xSemaphoreGive(gCameraMutex);
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  gCurrentFrameSize = frameSize;
  applyCameraSensorSettings(sensor);
  gCameraInitialized = true;
  delay(75);
  camera_fb_t* warmupFrame = esp_camera_fb_get();
  if (warmupFrame != nullptr) {
    const uint16_t warmupWidth = warmupFrame->width;
    const uint16_t warmupHeight = warmupFrame->height;
    const unsigned warmupLength = static_cast<unsigned>(warmupFrame->len);
    esp_camera_fb_return(warmupFrame);
    log_i("CAMERA warmup frame ok reason=%s size=%ux%u len=%u",
          reason != nullptr ? reason : "unknown", warmupWidth, warmupHeight, warmupLength);
  } else {
    log_e("CAMERA warmup frame failed reason=%s", reason != nullptr ? reason : "unknown");
    if (gOv5640RawWindowRuntimeEnabled) {
      log_e("CAMERA retrying without OV5640 raw window");
      gOv5640RawWindowRuntimeEnabled = false;
      esp_camera_deinit();
      gCameraInitialized = false;
      delay(75);
      config = makeCameraConfig();
      config.frame_size = frameSize;
      const esp_err_t retryInitResult = esp_camera_init(&config);
      if (retryInitResult != ESP_OK) {
        log_e("CAMERA retry init failed reason=%s frame_size=%d err=0x%x",
              reason != nullptr ? reason : "unknown", static_cast<int>(frameSize),
              static_cast<unsigned>(retryInitResult));
        xSemaphoreGive(gCameraMutex);
        return false;
      }

      sensor = esp_camera_sensor_get();
      applyCameraSensorSettings(sensor);
      gCameraInitialized = true;
      delay(75);
      warmupFrame = esp_camera_fb_get();
      if (warmupFrame != nullptr) {
        const uint16_t retryWarmupWidth = warmupFrame->width;
        const uint16_t retryWarmupHeight = warmupFrame->height;
        const unsigned retryWarmupLength = static_cast<unsigned>(warmupFrame->len);
        esp_camera_fb_return(warmupFrame);
        log_i("CAMERA retry warmup ok reason=%s size=%ux%u len=%u",
              reason != nullptr ? reason : "unknown", retryWarmupWidth, retryWarmupHeight,
              retryWarmupLength);
      } else {
        log_e("CAMERA retry warmup failed reason=%s", reason != nullptr ? reason : "unknown");
        xSemaphoreGive(gCameraMutex);
        return false;
      }
    }
  }
  xSemaphoreGive(gCameraMutex);
  log_i("CAMERA reinit complete reason=%s frame_size=%d",
        reason != nullptr ? reason : "unknown", static_cast<int>(frameSize));
  return true;
}

void setupCamera() {
  gHasPsram = psramFound();
  if (!reinitializeCameraForFrameSize(defaultCameraFrameSize(), "boot")) {
    blinkFatal();
  }
}

int readOv5640Reg16(sensor_t* sensor, int highReg, int lowReg) {
  const int high = sensor->get_reg(sensor, highReg, 0xff);
  const int low = sensor->get_reg(sensor, lowReg, 0xff);
  if (high < 0 || low < 0) {
    return -1;
  }
  return (high << 8) | low;
}

bool writeOv5640Reg16(sensor_t* sensor, int highReg, int lowReg, int value) {
  if (sensor->set_reg(sensor, highReg, 0xff, (value >> 8) & 0xff) != 0) {
    return false;
  }
  if (sensor->set_reg(sensor, lowReg, 0xff, value & 0xff) != 0) {
    return false;
  }
  return true;
}

void dumpOv5640WindowRegs(sensor_t* sensor, const char* label) {
  const int startX = readOv5640Reg16(sensor, 0x3800, 0x3801);
  const int startY = readOv5640Reg16(sensor, 0x3802, 0x3803);
  const int endX = readOv5640Reg16(sensor, 0x3804, 0x3805);
  const int endY = readOv5640Reg16(sensor, 0x3806, 0x3807);
  const int outputX = readOv5640Reg16(sensor, 0x3808, 0x3809);
  const int outputY = readOv5640Reg16(sensor, 0x380a, 0x380b);
  const int totalX = readOv5640Reg16(sensor, 0x380c, 0x380d);
  const int totalY = readOv5640Reg16(sensor, 0x380e, 0x380f);
  const int offsetX = readOv5640Reg16(sensor, 0x3810, 0x3811);
  const int offsetY = readOv5640Reg16(sensor, 0x3812, 0x3813);
  Serial.printf(
      "sensor regs %s start=(%d,%d) end=(%d,%d) output=%dx%d total=%dx%d offset=(%d,%d)\n",
      label != nullptr ? label : "window", startX, startY, endX, endY, outputX, outputY, totalX,
      totalY, offsetX, offsetY);
}

void dumpOv5640PresetRegisters(sensor_t* sensor, const char* label) {
  if (sensor == nullptr || static_cast<uint16_t>(sensor->id.PID) != OV5640_PID) {
    return;
  }

  struct RegPair {
    int high;
    int low;
    const char* name;
  };

  static constexpr RegPair kRegPairs[] = {
      {0x3800, 0x3801, "x_addr_st"},
      {0x3802, 0x3803, "y_addr_st"},
      {0x3804, 0x3805, "x_addr_end"},
      {0x3806, 0x3807, "y_addr_end"},
      {0x3808, 0x3809, "x_output_size"},
      {0x380a, 0x380b, "y_output_size"},
      {0x380c, 0x380d, "hts_total_x"},
      {0x380e, 0x380f, "vts_total_y"},
      {0x3810, 0x3811, "isp_x_offset"},
      {0x3812, 0x3813, "isp_y_offset"},
      {0x3814, 0x3815, "inc_xy"},
      {0x3816, 0x3817, "hsync_start"},
      {0x3818, 0x3819, "hsync_width"},
  };

  Serial.printf("sensor preset regs %s begin\n", label != nullptr ? label : "ov5640");
  for (const RegPair& reg : kRegPairs) {
    const int value = readOv5640Reg16(sensor, reg.high, reg.low);
    Serial.printf("sensor preset reg %s 0x%04x/0x%04x = 0x%04x (%d)\n", reg.name, reg.high,
                  reg.low, value >= 0 ? value : 0xffff, value);
  }

  static constexpr int kSingleRegs[] = {0x3818, 0x3820, 0x3821, 0x3034, 0x3035, 0x3036, 0x3037};
  for (const int reg : kSingleRegs) {
    const int value = sensor->get_reg(sensor, reg, 0xff);
    Serial.printf("sensor preset reg 0x%04x = 0x%02x (%d)\n", reg, value >= 0 ? value : 0xff,
                  value);
  }
  Serial.printf("sensor preset regs %s end\n", label != nullptr ? label : "ov5640");
}

bool applyOv5640RawWindow(sensor_t* sensor) {
  if (!gOv5640RawWindowRuntimeEnabled) {
    return false;
  }

  const uint16_t sensorPid = static_cast<uint16_t>(sensor->id.PID);
  if (sensorPid != OV5640_PID) {
    Serial.printf("sensor raw window skipped for sensor PID=0x%04x\n", sensorPid);
    return false;
  }

  const int startX = readOv5640Reg16(sensor, 0x3800, 0x3801);
  const int startY = readOv5640Reg16(sensor, 0x3802, 0x3803);
  const int endX = readOv5640Reg16(sensor, 0x3804, 0x3805);
  const int endY = readOv5640Reg16(sensor, 0x3806, 0x3807);
  const int offsetX = readOv5640Reg16(sensor, 0x3810, 0x3811);
  const int offsetY = readOv5640Reg16(sensor, 0x3812, 0x3813);
  const int totalX = readOv5640Reg16(sensor, 0x380c, 0x380d);
  const int totalY = readOv5640Reg16(sensor, 0x380e, 0x380f);
  const int incXy = readOv5640Reg16(sensor, 0x3814, 0x3815);
  const int hsyncStart = readOv5640Reg16(sensor, 0x3816, 0x3817);
  const int hsyncWidth = readOv5640Reg16(sensor, 0x3818, 0x3819);
  const int reg3818 = sensor->get_reg(sensor, 0x3818, 0xff);
  const int reg3820 = sensor->get_reg(sensor, 0x3820, 0xff);
  const int reg3821 = sensor->get_reg(sensor, 0x3821, 0xff);
  if (startX < 0 || startY < 0 || endX < 0 || endY < 0 || offsetX < 0 || offsetY < 0 ||
      totalX <= 0 || totalY <= 0 || incXy < 0 || hsyncStart < 0 || hsyncWidth < 0 ||
      reg3818 < 0 || reg3820 < 0 || reg3821 < 0) {
    Serial.println("sensor raw window failed to read OV5640 timing registers");
    return false;
  }

  const int currentWidth = endX - startX + 1;
  const int currentHeight = endY - startY + 1;
  if (currentWidth < 2 || currentHeight < 2) {
    Serial.println("sensor raw window invalid current dimensions");
    return false;
  }

  int windowWidth = std::max(2, kOv5640WindowWidthPixels);
  int windowHeight = std::max(2, kOv5640WindowHeightPixels);
  windowWidth &= ~1;
  windowHeight &= ~1;
  windowWidth = std::clamp(windowWidth, 2, currentWidth & ~1);
  windowHeight = std::clamp(windowHeight, 2, currentHeight & ~1);

  int presetWidth = 0;
  int presetHeight = 0;
  const bool hasPresetResolution = frameSizeToResolution(gCurrentFrameSize, presetWidth, presetHeight);
  if (kOv5640WindowLockPresetAspect && hasPresetResolution && presetWidth > 0 && presetHeight > 0) {
    const float presetAspect = static_cast<float>(presetWidth) / static_cast<float>(presetHeight);
    const float boundedAspect = static_cast<float>(windowWidth) / static_cast<float>(windowHeight);
    if (boundedAspect > presetAspect) {
      windowWidth = std::max(2, static_cast<int>(windowHeight * presetAspect + 0.5f)) & ~1;
    } else {
      windowHeight = std::max(2, static_cast<int>(windowWidth / presetAspect + 0.5f)) & ~1;
    }
    windowWidth = std::clamp(windowWidth, 2, currentWidth & ~1);
    windowHeight = std::clamp(windowHeight, 2, currentHeight & ~1);
  }

  const int centerX = startX + static_cast<int>(currentWidth * std::clamp(kOv5640WindowCenterXFraction, 0.0f, 1.0f));
  const int centerY = startY + static_cast<int>(currentHeight * std::clamp(kOv5640WindowCenterYFraction, 0.0f, 1.0f));

  int newStartX = centerX - windowWidth / 2;
  int newStartY = centerY - windowHeight / 2;
  int newEndX = newStartX + windowWidth - 1;
  int newEndY = newStartY + windowHeight - 1;

  if (newStartX < startX) {
    newEndX += startX - newStartX;
    newStartX = startX;
  }
  if (newStartY < startY) {
    newEndY += startY - newStartY;
    newStartY = startY;
  }
  if (newEndX > endX) {
    const int delta = newEndX - endX;
    newStartX = std::max(startX, newStartX - delta);
    newEndX = endX;
  }
  if (newEndY > endY) {
    const int delta = newEndY - endY;
    newStartY = std::max(startY, newStartY - delta);
    newEndY = endY;
  }

  newStartX &= ~1;
  newStartY &= ~1;
  newEndX |= 1;
  newEndY |= 1;

  const int croppedWidth = newEndX - newStartX + 1;
  const int croppedHeight = newEndY - newStartY + 1;

  const int defaultOutputWidth =
      kOv5640WindowUseCroppedOutput ? croppedWidth : (hasPresetResolution ? presetWidth : croppedWidth);
  const int defaultOutputHeight =
      kOv5640WindowUseCroppedOutput ? croppedHeight : (hasPresetResolution ? presetHeight : croppedHeight);
  int outputX = kOv5640WindowOutputWidth > 0 ? kOv5640WindowOutputWidth : defaultOutputWidth;
  int outputY = kOv5640WindowOutputHeight > 0 ? kOv5640WindowOutputHeight : defaultOutputHeight;
  if (hasPresetResolution) {
    outputX = std::min(outputX, presetWidth);
    outputY = std::min(outputY, presetHeight);
  }
  outputX = std::max(2, outputX & ~1);
  outputY = std::max(2, outputY & ~1);

  const bool scale = outputX != croppedWidth || outputY != croppedHeight;
  dumpOv5640WindowRegs(sensor, "before");

  if (kUseOv5640DirectWindowRegisters) {
    bool ok = true;
    ok = ok && writeOv5640Reg16(sensor, 0x3800, 0x3801, newStartX);
    ok = ok && writeOv5640Reg16(sensor, 0x3802, 0x3803, newStartY);
    ok = ok && writeOv5640Reg16(sensor, 0x3804, 0x3805, newEndX);
    ok = ok && writeOv5640Reg16(sensor, 0x3806, 0x3807, newEndY);
    ok = ok && writeOv5640Reg16(sensor, 0x3808, 0x3809, outputX);
    ok = ok && writeOv5640Reg16(sensor, 0x380a, 0x380b, outputY);
    ok = ok && writeOv5640Reg16(sensor, 0x3810, 0x3811, 0);
    ok = ok && writeOv5640Reg16(sensor, 0x3812, 0x3813, 0);
    ok = ok && writeOv5640Reg16(sensor, 0x3814, 0x3815, 0x1111);
    ok = ok && writeOv5640Reg16(sensor, 0x3816, 0x3817, hsyncStart);
    ok = ok && writeOv5640Reg16(sensor, 0x3818, 0x3819, hsyncWidth);
    ok = ok && sensor->set_reg(sensor, 0x3818, 0xff, reg3818) == 0;
    ok = ok && sensor->set_reg(sensor, 0x3820, 0xff, 0x40) == 0;
    ok = ok && sensor->set_reg(sensor, 0x3821, 0xff, 0x06) == 0;
    if (!ok) {
      Serial.println("sensor raw window direct register write failed");
      return false;
    }
    dumpOv5640WindowRegs(sensor, "after_direct");
    Serial.printf(
        "sensor raw window direct start=(%d,%d) end=(%d,%d) cropped=%dx%d output=%dx%d scale=%d cropped_output=%d lock_preset_aspect=%d preset=%dx%d total=%dx%d\n",
        newStartX, newStartY, newEndX, newEndY, croppedWidth, croppedHeight, outputX, outputY,
        scale ? 1 : 0, kOv5640WindowUseCroppedOutput ? 1 : 0,
        kOv5640WindowLockPresetAspect ? 1 : 0, presetWidth, presetHeight, totalX, totalY);
    return true;
  }

  const int rc = sensor->set_res_raw(sensor, newStartX, newStartY, newEndX, newEndY, offsetX,
                                     offsetY, totalX, totalY, outputX, outputY, scale, false);
  dumpOv5640WindowRegs(sensor, "after_set_res_raw");
  Serial.printf(
      "sensor raw window rc=%d start=(%d,%d) end=(%d,%d) cropped=%dx%d output=%dx%d scale=%d cropped_output=%d lock_preset_aspect=%d preset=%dx%d total=%dx%d\n",
      rc, newStartX, newStartY, newEndX, newEndY, croppedWidth, croppedHeight, outputX, outputY,
      scale ? 1 : 0, kOv5640WindowUseCroppedOutput ? 1 : 0, kOv5640WindowLockPresetAspect ? 1 : 0,
      presetWidth, presetHeight, totalX, totalY);
  return rc == 0;
}

bool applyOv5640SensorDigitalZoom(sensor_t* sensor) {
  if (!kEnableOv5640SensorDigitalZoom || kOv5640SensorDigitalZoomFactor <= 1.0f) {
    return false;
  }

  const uint16_t sensorPid = static_cast<uint16_t>(sensor->id.PID);
  if (sensorPid != OV5640_PID) {
    Serial.printf("sensor zoom skipped for sensor PID=0x%04x\n", sensorPid);
    return false;
  }

  const int startX = readOv5640Reg16(sensor, 0x3800, 0x3801);
  const int startY = readOv5640Reg16(sensor, 0x3802, 0x3803);
  const int endX = readOv5640Reg16(sensor, 0x3804, 0x3805);
  const int endY = readOv5640Reg16(sensor, 0x3806, 0x3807);
  const int offsetX = readOv5640Reg16(sensor, 0x3810, 0x3811);
  const int offsetY = readOv5640Reg16(sensor, 0x3812, 0x3813);
  const int outputX = readOv5640Reg16(sensor, 0x3808, 0x3809);
  const int outputY = readOv5640Reg16(sensor, 0x380a, 0x380b);
  const int totalX = readOv5640Reg16(sensor, 0x380c, 0x380d);
  const int totalY = readOv5640Reg16(sensor, 0x380e, 0x380f);
  if (startX < 0 || startY < 0 || endX < 0 || endY < 0 || offsetX < 0 || offsetY < 0 ||
      outputX <= 0 || outputY <= 0 || totalX <= 0 || totalY <= 0) {
    Serial.println("sensor zoom failed to read OV5640 timing registers");
    return false;
  }

  const int currentWidth = endX - startX + 1;
  const int currentHeight = endY - startY + 1;
  const int newWidth =
      std::max(outputX, static_cast<int>(currentWidth / kOv5640SensorDigitalZoomFactor + 0.5f));
  const int newHeight =
      std::max(outputY, static_cast<int>(currentHeight / kOv5640SensorDigitalZoomFactor + 0.5f));

  int adjustedWidth = newWidth & ~1;
  int adjustedHeight = newHeight & ~1;
  if (adjustedWidth <= 0 || adjustedHeight <= 0) {
    Serial.println("sensor zoom invalid adjusted crop");
    return false;
  }

  const int centerX = startX + currentWidth / 2;
  const int centerY = startY + currentHeight / 2;
  int newStartX = centerX - adjustedWidth / 2;
  int newStartY = centerY - adjustedHeight / 2;
  int newEndX = newStartX + adjustedWidth - 1;
  int newEndY = newStartY + adjustedHeight - 1;

  if (newStartX < 0) {
    newEndX -= newStartX;
    newStartX = 0;
  }
  if (newStartY < 0) {
    newEndY -= newStartY;
    newStartY = 0;
  }
  if (newEndX > endX) {
    const int delta = newEndX - endX;
    newStartX = std::max(0, newStartX - delta);
    newEndX = endX;
  }
  if (newEndY > endY) {
    const int delta = newEndY - endY;
    newStartY = std::max(0, newStartY - delta);
    newEndY = endY;
  }

  newStartX &= ~1;
  newStartY &= ~1;
  newEndX |= 1;
  newEndY |= 1;

  const int rc = sensor->set_res_raw(sensor, newStartX, newStartY, newEndX, newEndY, offsetX,
                                     offsetY, totalX, totalY, outputX, outputY, true, false);
  Serial.printf(
      "sensor zoom factor=%.2f rc=%d start=(%d,%d) end=(%d,%d) output=%dx%d total=%dx%d\n",
      kOv5640SensorDigitalZoomFactor, rc, newStartX, newStartY, newEndX, newEndY, outputX,
      outputY, totalX, totalY);
  return rc == 0;
}

bool prepareEdgeImpulseInput(const camera_fb_t* fb, EdgeImpulseTransform& transform) {
  if (!kEnableEdgeImpulseFirstLine) {
    return false;
  }
  if (gEiInputBuffer == nullptr) {
    gEiInputBuffer = static_cast<uint8_t*>(
        malloc(EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3));
    if (gEiInputBuffer == nullptr) {
      Serial.println("EI input buffer alloc failed");
      return false;
    }
  }

  const int srcWidth = static_cast<int>(fb->width);
  const int srcHeight = static_cast<int>(fb->height);
  const int roiWidth =
      std::clamp(static_cast<int>(srcWidth * kEiCropWidthFraction + 0.5f), 1, srcWidth);
  const int roiHeight =
      std::clamp(static_cast<int>(srcHeight * kEiCropHeightFraction + 0.5f), 1, srcHeight);
  const int roiCenterX = std::clamp(static_cast<int>(srcWidth * kEiCropCenterXFraction + 0.5f), 0,
                                    srcWidth - 1);
  const int roiCenterY = std::clamp(static_cast<int>(srcHeight * kEiCropCenterYFraction + 0.5f), 0,
                                    srcHeight - 1);
  const int roiX = std::clamp(roiCenterX - roiWidth / 2, 0, srcWidth - roiWidth);
  const int roiY = std::clamp(roiCenterY - roiHeight / 2, 0, srcHeight - roiHeight);
  const int topTrim = std::clamp(static_cast<int>(roiHeight * 0.30f + 0.5f), 0, roiHeight - 1);
  const int effectiveRoiY = roiY + topTrim;
  const int effectiveRoiHeight = std::max(1, roiHeight - topTrim);
  const float scale = std::max(
      static_cast<float>(EI_CLASSIFIER_INPUT_WIDTH) / static_cast<float>(roiWidth),
      static_cast<float>(EI_CLASSIFIER_INPUT_HEIGHT) / static_cast<float>(effectiveRoiHeight));
  const int resizedWidth = std::max(EI_CLASSIFIER_INPUT_WIDTH,
                                    static_cast<int>(roiWidth * scale + 0.5f));
  const int resizedHeight = std::max(EI_CLASSIFIER_INPUT_HEIGHT,
                                     static_cast<int>(effectiveRoiHeight * scale + 0.5f));
  const int cropX = std::max(0, (resizedWidth - EI_CLASSIFIER_INPUT_WIDTH) / 2);
  const int cropY = std::max(0, (resizedHeight - EI_CLASSIFIER_INPUT_HEIGHT) / 2);

  transform.roiX = roiX;
  transform.roiY = effectiveRoiY;
  transform.roiWidth = roiWidth;
  transform.roiHeight = effectiveRoiHeight;
  transform.scale = scale;
  transform.resizedWidth = resizedWidth;
  transform.resizedHeight = resizedHeight;
  transform.cropX = cropX;
  transform.cropY = cropY;

  for (int y = 0; y < EI_CLASSIFIER_INPUT_HEIGHT; ++y) {
    for (int x = 0; x < EI_CLASSIFIER_INPUT_WIDTH; ++x) {
      const int resizedX = x + cropX;
      const int resizedY = y + cropY;
      const int srcX =
          std::min(srcWidth - 1, roiX + static_cast<int>(resizedX / scale));
      const int srcY =
          std::min(srcHeight - 1, effectiveRoiY + static_cast<int>(resizedY / scale));
      int r = 0;
      int g = 0;
      int b = 0;
      unpackRgb565(fb->buf, (srcY * srcWidth + srcX) * 2, r, g, b);
      const size_t outIndex =
          static_cast<size_t>((y * EI_CLASSIFIER_INPUT_WIDTH + x) * 3);
      gEiInputBuffer[outIndex + 0] = static_cast<uint8_t>(r);
      gEiInputBuffer[outIndex + 1] = static_cast<uint8_t>(g);
      gEiInputBuffer[outIndex + 2] = static_cast<uint8_t>(b);
    }
  }

  return true;
}

static int edgeImpulseGetData(size_t offset, size_t length, float* outPtr) {
  const size_t totalPixels = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  if (offset >= totalPixels) return -1;
  if (offset + length > totalPixels) length = totalPixels - offset;
  size_t pixelIndex = offset * 3;
  for (size_t outIndex = 0; outIndex < length; ++outIndex) {
    outPtr[outIndex] = (gEiInputBuffer[pixelIndex] << 16) |
                       (gEiInputBuffer[pixelIndex + 1] << 8) |
                       gEiInputBuffer[pixelIndex + 2];
    pixelIndex += 3;
  }
  return 0;
}

bool analyzeTrafficLightWithEdgeImpulse(const camera_fb_t* fb, TrafficLightState& state) {
  if (!kEnableEdgeImpulseFirstLine) {
    return false;
  }

  EdgeImpulseTransform transform;
  if (!prepareEdgeImpulseInput(fb, transform)) {
    return false;
  }

  state.hasEiCrop = true;
  state.eiCropX = transform.roiX;
  state.eiCropY = transform.roiY;
  state.eiCropWidth = transform.roiWidth;
  state.eiCropHeight = transform.roiHeight;
  state.eiScale = transform.scale;
  state.eiResizedWidth = transform.resizedWidth;
  state.eiResizedHeight = transform.resizedHeight;
  state.eiInputCropX = transform.cropX;
  state.eiInputCropY = transform.cropY;

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &edgeImpulseGetData;

  ei_impulse_result_t result = {0};
  const uint32_t inferenceStartMs = millis();
  const EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
  const uint32_t inferenceDurationMs = millis() - inferenceStartMs;
  state.rawEiOutput.clear();
  state.rawEiOutput.reserve(512);
  state.rawEiOutput += "ei_result=" + std::to_string(static_cast<int>(err)) + "\n";
  state.rawEiOutput += "inference_duration_ms=" + std::to_string(inferenceDurationMs) + "\n";
  state.rawEiOutput += "bounding_boxes_count=" +
                       std::to_string(static_cast<unsigned>(result.bounding_boxes_count)) + "\n";
  state.rawEiOutput += "ei_crop_x=" + std::to_string(transform.roiX) + "\n";
  state.rawEiOutput += "ei_crop_y=" + std::to_string(transform.roiY) + "\n";
  state.rawEiOutput += "ei_crop_width=" + std::to_string(transform.roiWidth) + "\n";
  state.rawEiOutput += "ei_crop_height=" + std::to_string(transform.roiHeight) + "\n";
  state.rawEiOutput += "ei_scale=" + std::to_string(transform.scale) + "\n";
  state.rawEiOutput += "ei_resized_width=" + std::to_string(transform.resizedWidth) + "\n";
  state.rawEiOutput += "ei_resized_height=" + std::to_string(transform.resizedHeight) + "\n";
  state.rawEiOutput += "ei_input_crop_x=" + std::to_string(transform.cropX) + "\n";
  state.rawEiOutput += "ei_input_crop_y=" + std::to_string(transform.cropY) + "\n";
  Serial.printf(
      "EI inference duration=%lu ms result=%d boxes=%u\n",
      static_cast<unsigned long>(inferenceDurationMs),
      static_cast<int>(err),
      static_cast<unsigned int>(result.bounding_boxes_count));
  if (err != EI_IMPULSE_OK) {
    Serial.printf("EI run_classifier failed=%d\n", static_cast<int>(err));
    return false;
  }

  float bestScore = 0.0f;
  ei_impulse_result_bounding_box_t bestBox{};
  bool found = false;
  int bestTopY = 0;
  int bestPaddedX1 = 0;
  int bestPaddedY1 = 0;
  int bestPaddedX2 = 0;
  int bestPaddedY2 = 0;
  int bestClusterCenterX = 0;
  int bestClusterCenterY = 0;
  for (uint32_t i = 0; i < result.bounding_boxes_count; ++i) {
    const auto& bb = result.bounding_boxes[i];
    state.rawEiOutput += "box_" + std::to_string(i) + "_label=" + std::string(bb.label) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_raw_x=" + std::to_string(static_cast<int>(bb.x)) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_raw_y=" + std::to_string(static_cast<int>(bb.y)) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_raw_width=" + std::to_string(static_cast<int>(bb.width)) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_raw_height=" + std::to_string(static_cast<int>(bb.height)) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_confidence=" + std::to_string(bb.value) + "\n";
    if (bb.value < kEdgeImpulseMinConfidence) {
      state.rawEiOutput += "box_" + std::to_string(i) + "_rejected=low_confidence\n";
      continue;
    }
    const float boxSrcX1 = static_cast<float>(transform.roiX) +
                           (static_cast<float>(bb.x) + static_cast<float>(transform.cropX)) /
                               transform.scale;
    const float boxSrcY1 = static_cast<float>(transform.roiY) +
                           (static_cast<float>(bb.y) + static_cast<float>(transform.cropY)) /
                               transform.scale;
    const float boxSrcX2 =
        static_cast<float>(transform.roiX) +
        (static_cast<float>(bb.x + bb.width) + static_cast<float>(transform.cropX)) /
            transform.scale;
    const float boxSrcY2 =
        static_cast<float>(transform.roiY) +
        (static_cast<float>(bb.y + bb.height) + static_cast<float>(transform.cropY)) /
            transform.scale;
    const int projectedX1 = std::max(0, static_cast<int>(std::floor(boxSrcX1)));
    const int projectedY1 = std::max(0, static_cast<int>(std::floor(boxSrcY1)));
    const int projectedX2 = std::min(static_cast<int>(fb->width) - 1,
                                     static_cast<int>(std::ceil(boxSrcX2)) - 1);
    const int projectedY2 = std::min(static_cast<int>(fb->height) - 1,
                                     static_cast<int>(std::ceil(boxSrcY2)) - 1);
    int paddedProjectedX1 = projectedX1;
    int paddedProjectedY1 = projectedY1;
    int paddedProjectedX2 = projectedX2;
    int paddedProjectedY2 = projectedY2;
    padBoundingBox(paddedProjectedX1, paddedProjectedY1, paddedProjectedX2, paddedProjectedY2,
                   static_cast<int>(fb->width), static_cast<int>(fb->height));
    RedClusterCheckResult clusterCheck =
        evaluateRedClusterInBox(fb, paddedProjectedX1, paddedProjectedY1, paddedProjectedX2, paddedProjectedY2);
    for (int expansionStep = 0; expansionStep < kEiClusterEdgeExpansionSteps; ++expansionStep) {
      const int currentWidth = paddedProjectedX2 - paddedProjectedX1 + 1;
      const int currentHeight = paddedProjectedY2 - paddedProjectedY1 + 1;
      if (!clusterTouchesBoxEdge(clusterCheck, currentWidth, currentHeight)) {
        break;
      }
      const int previousX1 = paddedProjectedX1;
      const int previousY1 = paddedProjectedY1;
      const int previousX2 = paddedProjectedX2;
      const int previousY2 = paddedProjectedY2;
      expandBoundingBox(paddedProjectedX1, paddedProjectedY1, paddedProjectedX2, paddedProjectedY2,
                        static_cast<int>(fb->width), static_cast<int>(fb->height));
      if (paddedProjectedX1 == previousX1 && paddedProjectedY1 == previousY1 &&
          paddedProjectedX2 == previousX2 && paddedProjectedY2 == previousY2) {
        break;
      }
      clusterCheck = evaluateRedClusterInBox(
          fb, paddedProjectedX1, paddedProjectedY1, paddedProjectedX2, paddedProjectedY2);
    }
    state.rawEiOutput += "box_" + std::to_string(i) + "_projected_x_min=" +
                         std::to_string(projectedX1) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_projected_y_min=" +
                         std::to_string(projectedY1) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_projected_x_max=" +
                         std::to_string(projectedX2) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_projected_y_max=" +
                         std::to_string(projectedY2) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_padded_x_min=" +
                         std::to_string(paddedProjectedX1) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_padded_y_min=" +
                         std::to_string(paddedProjectedY1) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_padded_x_max=" +
                         std::to_string(paddedProjectedX2) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_padded_y_max=" +
                         std::to_string(paddedProjectedY2) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_warm_pixels=" +
                         std::to_string(clusterCheck.warmPixels) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_largest_warm_cluster=" +
                         std::to_string(clusterCheck.largestCluster) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_cluster_min_x=" +
                         std::to_string(clusterCheck.clusterMinX) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_cluster_min_y=" +
                         std::to_string(clusterCheck.clusterMinY) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_cluster_max_x=" +
                         std::to_string(clusterCheck.clusterMaxX) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_cluster_max_y=" +
                         std::to_string(clusterCheck.clusterMaxY) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_cluster_center_x=" +
                         std::to_string(clusterCheck.clusterCenterX) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_cluster_center_y=" +
                         std::to_string(clusterCheck.clusterCenterY) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_core_min_x=" +
                         std::to_string(clusterCheck.coreMinX) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_core_min_y=" +
                         std::to_string(clusterCheck.coreMinY) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_core_max_x=" +
                         std::to_string(clusterCheck.coreMaxX) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_core_max_y=" +
                         std::to_string(clusterCheck.coreMaxY) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_brightest_x=" +
                         std::to_string(clusterCheck.brightestX) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_brightest_y=" +
                         std::to_string(clusterCheck.brightestY) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_brightest_luma=" +
                         std::to_string(clusterCheck.brightestLuma) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_cluster_aspect_ratio=" +
                         std::to_string(clusterCheck.clusterAspectRatio) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_cluster_fill_fraction=" +
                         std::to_string(clusterCheck.clusterFillFraction) + "\n";
    state.rawEiOutput += "box_" + std::to_string(i) + "_cluster_accepted=" +
                         std::to_string(clusterCheck.accepted ? 1 : 0) + "\n";
    if (!clusterCheck.accepted) {
      state.rawEiOutput += "box_" + std::to_string(i) + "_rejected=cluster_not_round_enough\n";
      log_d("EI reject box_%u conf=%.3f projected=(%d,%d,%d,%d) padded=(%d,%d,%d,%d) warm_pixels=%d largest_cluster=%d aspect=%.3f fill=%.3f",
            static_cast<unsigned>(i), bb.value, projectedX1, projectedY1, projectedX2, projectedY2,
            paddedProjectedX1, paddedProjectedY1, paddedProjectedX2, paddedProjectedY2,
            clusterCheck.warmPixels, clusterCheck.largestCluster, clusterCheck.clusterAspectRatio,
            clusterCheck.clusterFillFraction);
      continue;
    }
    log_d("EI accept box_%u conf=%.3f projected=(%d,%d,%d,%d) padded=(%d,%d,%d,%d) warm_pixels=%d largest_cluster=%d aspect=%.3f fill=%.3f",
          static_cast<unsigned>(i), bb.value, projectedX1, projectedY1, projectedX2, projectedY2,
          paddedProjectedX1, paddedProjectedY1, paddedProjectedX2, paddedProjectedY2,
          clusterCheck.warmPixels, clusterCheck.largestCluster, clusterCheck.clusterAspectRatio,
          clusterCheck.clusterFillFraction);
    const redbox::SelectedRedRegion selectedRegion = redbox::selectRedRegionFromCluster(
        paddedProjectedX1, paddedProjectedY1, paddedProjectedX2, paddedProjectedY2, clusterCheck,
        static_cast<int>(fb->width), static_cast<int>(fb->height));
    if (state.serializedBoxCount < kMaxSerializedBoxes) {
      SerializableBoundingBox& outBox = state.serializedBoxes[state.serializedBoxCount++];
      std::snprintf(outBox.label, sizeof(outBox.label), "%s", bb.label);
      outBox.rawX = static_cast<int>(bb.x);
      outBox.rawY = static_cast<int>(bb.y);
      outBox.rawWidth = static_cast<int>(bb.width);
      outBox.rawHeight = static_cast<int>(bb.height);
      outBox.xMin = selectedRegion.minX;
      outBox.yMin = selectedRegion.minY;
      outBox.xMax = selectedRegion.maxX;
      outBox.yMax = selectedRegion.maxY;
      outBox.clusterCenterX = selectedRegion.centerX;
      outBox.clusterCenterY = selectedRegion.centerY;
      outBox.confidence = bb.value;
    }
    if (!found || bb.y < bestTopY || (bb.y == bestTopY && bb.value > bestScore)) {
      bestScore = bb.value;
      bestBox = bb;
      bestTopY = bb.y;
      bestPaddedX1 = selectedRegion.minX;
      bestPaddedY1 = selectedRegion.minY;
      bestPaddedX2 = selectedRegion.maxX;
      bestPaddedY2 = selectedRegion.maxY;
      bestClusterCenterX = selectedRegion.centerX;
      bestClusterCenterY = selectedRegion.centerY;
      found = true;
    }
  }

  log_i("EI found=%d best=%.3f serialized_boxes=%u total_boxes=%u", found ? 1 : 0, bestScore,
        static_cast<unsigned>(state.serializedBoxCount),
        static_cast<unsigned>(result.bounding_boxes_count));
  if (!found) {
    return false;
  }

  state.redFound = true;
  state.redFoundByEi = true;
  state.redOn = true;
  state.eiConfidence = bestScore;
  state.red.found = true;
  state.red.min_x = bestPaddedX1;
  state.red.min_y = bestPaddedY1;
  state.red.max_x = bestPaddedX2;
  state.red.max_y = bestPaddedY2;
  state.red.center_x = bestClusterCenterX;
  state.red.center_y = bestClusterCenterY;
  state.red.radius = std::max(2, std::max(bestPaddedX2 - bestPaddedX1, bestPaddedY2 - bestPaddedY1) / 2);
  state.red.pixel_count = 0;
  return true;
}

void setupAutofocus() {
  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    Serial.println("esp_camera_sensor_get returned null");
    return;
  }

  Serial.println("AUTOFOCUS setup begin");
  Serial.printf("sensor PID=0x%02x VER=0x%02x MIDH=0x%02x MIDL=0x%02x\n",
                sensor->id.PID, sensor->id.VER, sensor->id.MIDH, sensor->id.MIDL);

  const int aeLevelResult = sensor->set_ae_level(sensor, kAeLevel);
  gCurrentAeLevel = kAeLevel;
  Serial.printf("sensor ae_level=%d rc=%d\n", kAeLevel, aeLevelResult);

  const bool sensorZoomApplied = applyOv5640SensorDigitalZoom(sensor);
  Serial.printf("sensor digital zoom applied=%d\n", sensorZoomApplied ? 1 : 0);

  if (!kEnableOv5640Autofocus) {
    Serial.println("OV5640 autofocus disabled in config");
    return;
  }

  const uint16_t sensorPid = static_cast<uint16_t>(sensor->id.PID);
  if (sensorPid != OV5640_PID) {
    Serial.printf("OV5640 autofocus skipped for sensor PID=0x%04x\n", sensorPid);
    return;
  }

  Serial.printf("ov5640.getFWStatus before start => 0x%02x\n", ov5640.getFWStatus());
  Serial.println("ov5640.start begin");
  ov5640.start(sensor);
  Serial.printf("ov5640.getFWStatus after start => 0x%02x\n", ov5640.getFWStatus());

  Serial.println("ov5640.focusInit begin");
  const int focusInitResult = ov5640.focusInit();
  Serial.printf("ov5640.focusInit => %d\n", focusInitResult);
  Serial.printf("ov5640.getFWStatus after focusInit => 0x%02x\n", ov5640.getFWStatus());
  if (focusInitResult != 0) {
    return;
  }

  Serial.printf("ov5640 settle delay ms=%lu begin\n",
                static_cast<unsigned long>(kAutofocusSettleDelayMs));
  delay(kAutofocusSettleDelayMs);
  Serial.printf("ov5640.getFWStatus after settle => 0x%02x\n", ov5640.getFWStatus());

  Serial.println("ov5640.autoFocusMode begin");
  const int autoFocusResult = ov5640.autoFocusMode();
  Serial.printf("ov5640.autoFocusMode => %d\n", autoFocusResult);
  Serial.printf("ov5640.getFWStatus after autoFocusMode => 0x%02x\n", ov5640.getFWStatus());
  Serial.println("AUTOFOCUS setup end");
}

void waitForAutofocusFocused() {
  if (!kEnableOv5640Autofocus || !kWaitForOv5640FocusedBeforeCapture) {
    return;
  }

  const uint32_t startMs = millis();
  while (millis() - startMs < kAutofocusFocusTimeoutMs) {
    const uint8_t status = ov5640.getFWStatus();
    if (status == FW_STATUS_S_FOCUSED) {
      return;
    }
    delay(kAutofocusPollIntervalMs);
  }

  Serial.println("AUTOFOCUS focus wait timeout");
}

TrafficLightState analyzeTrafficLight(const camera_fb_t* fb) {
  TrafficLightState state;
  const int frameHeight = static_cast<int>(fb->height);
  if (!useBackupAlgo) {
    gHasLockedTarget = false;
    gRedLockMissingFrames = 0;
  }
  Serial.printf("ANALYZE frame=%ux%u locked=%d\n", fb->width, fb->height, gHasLockedTarget ? 1 : 0);

  if (!gHasLockedTarget) {
    if (analyzeTrafficLightWithEdgeImpulse(fb, state)) {
      Serial.printf("ANALYZE EI result red_found=%d center=(%d,%d) pixels=%d\n",
                    state.redFound ? 1 : 0, state.red.center_x, state.red.center_y,
                    state.red.pixel_count);
      return state;
    }

    if (!useBackupAlgo) {
      Serial.println("ANALYZE EI miss and backup disabled");
      return state;
    }

    const traffic::Rgb565Frame frame{fb->buf, static_cast<int>(fb->width), frameHeight};
    const auto detection = traffic::analyze_frame(frame);
    state.red = detection.red;
    state.redFound = detection.red_found;
    state.redOn = detection.red_found;
    Serial.printf("ANALYZE result red_found=%d center=(%d,%d) pixels=%d\n",
                  state.redFound ? 1 : 0, state.red.center_x, state.red.center_y,
                  state.red.pixel_count);
    return state;
  }

  state.redFound = true;
  state.redLocked = true;
  state.redOn = true;
  state.red = gLockedRed;

  if (!useBackupAlgo) {
    return state;
  }

  const traffic::Rgb565Frame frame{fb->buf, static_cast<int>(fb->width), frameHeight};
  const auto lockCheck = traffic::analyze_frame(frame);
  const bool redStillPresent = lockCheck.red_found &&
                               abs(lockCheck.red.center_x - state.red.center_x) <= kRedCenterMoveTolerance &&
                               abs(lockCheck.red.center_y - state.red.center_y) <= kRedCenterMoveTolerance;
  if (!redStillPresent) {
    ++gRedLockMissingFrames;
  } else {
    gRedLockMissingFrames = 0;
  }

  if (gRedLockMissingFrames >= kRedLockDarkFramesToRelease) {
    state.redOn = false;
    gHasLockedTarget = false;
    gRedLockMissingFrames = 0;
    gStableRedFrames = 0;
    gRedSnapshotLatched = false;
  }

  Serial.printf("ANALYZE lock redStillPresent=%d missing=%d redOn=%d\n", redStillPresent ? 1 : 0,
                gRedLockMissingFrames, state.redOn ? 1 : 0);

  return state;
}

bool shouldSendSnapshots(const TrafficLightState& state, uint32_t nowMs) {
  if (!state.redFound) {
    gStableRedFrames = 0;
    gRedSnapshotLatched = false;
    gLastRedCenterX = -1000;
    gLastRedCenterY = -1000;
    Serial.println("SNAPSHOT skip red not found");
    return false;
  }

  const int dx = state.red.center_x - gLastRedCenterX;
  const int dy = state.red.center_y - gLastRedCenterY;
  const bool sameLamp =
      (abs(dx) <= kRedCenterMoveTolerance && abs(dy) <= kRedCenterMoveTolerance);

  if (sameLamp) {
    ++gStableRedFrames;
  } else {
    gStableRedFrames = 1;
    gRedSnapshotLatched = false;
  }

  gLastRedCenterX = state.red.center_x;
  gLastRedCenterY = state.red.center_y;

  if (gStableRedFrames < kStableFramesRequired) {
    Serial.printf("SNAPSHOT wait stable=%d required=%d\n", gStableRedFrames, kStableFramesRequired);
    return false;
  }

  if (useBackupAlgo && !gHasLockedTarget) {
    gHasLockedTarget = true;
    gLockedRed = state.red;
    gRedLockMissingFrames = 0;
  }

  if (!gRedSnapshotLatched || nowMs - gLastSnapshotMs >= kSnapshotCooldownMs) {
    gRedSnapshotLatched = true;
    gLastSnapshotMs = nowMs;
    Serial.printf("SNAPSHOT trigger now=%lu center=(%d,%d)\n", static_cast<unsigned long>(nowMs),
                  state.red.center_x, state.red.center_y);
    return true;
  }

  Serial.printf("SNAPSHOT cooldown remaining=%lu\n",
                static_cast<unsigned long>(kSnapshotCooldownMs - (nowMs - gLastSnapshotMs)));
  return false;
}

void reportTrafficLightState(const TrafficLightState& state, uint32_t nowMs) {
  if (!state.redFound) {
    Serial.println("TRAFFIC_LIGHT red_found=0");
    setCurrentBleStatus("Monitoring", "Scanning for a red light.", false, false, nowMs, true);
    return;
  }

  Serial.printf(
      "TRAFFIC_LIGHT red_found=1 red_locked=%d red_on=%d red_center=(%d,%d) red_box=(%d,%d,%d,%d) red_pixels=%d\n",
      state.redLocked ? 1 : 0, state.redOn ? 1 : 0, state.red.center_x, state.red.center_y,
      state.red.min_x, state.red.min_y, state.red.max_x, state.red.max_y, state.red.pixel_count);
  setCurrentBleStatus("Red detected", "Locked onto a red signal and monitoring for a change.", true,
                      false, nowMs, true);
}

bool writeJpegPacket(ImageType imageType, uint16_t width, uint16_t height, uint8_t* jpegData,
                     size_t jpegSize) {
  const uint32_t sequence = gPacketSequence++;
  ImagePacketHeader header{{'T', 'L', 'I', 'M'},
                           1,
                           imageType,
                           width,
                           height,
                           sequence,
                           static_cast<uint32_t>(jpegSize)};
  const size_t headerBytes = Serial.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
  const size_t payloadBytes = Serial.write(jpegData, jpegSize);
  Serial.flush();
  return headerBytes == sizeof(header) && payloadBytes == jpegSize;
}

bool writeTextPacket(ImageType imageType, uint16_t width, uint16_t height, uint32_t sequence,
                     const std::string& text) {
  ImagePacketHeader header{{'T', 'L', 'I', 'M'},
                           1,
                           imageType,
                           width,
                           height,
                           sequence,
                           static_cast<uint32_t>(text.size())};
  const size_t headerBytes = Serial.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
  const size_t payloadBytes =
      Serial.write(reinterpret_cast<const uint8_t*>(text.data()), text.size());
  Serial.flush();
  return headerBytes == sizeof(header) && payloadBytes == text.size();
}

bool writeBinaryPacket(ImageType imageType, uint16_t width, uint16_t height, uint32_t sequence,
                       const uint8_t* data, size_t size) {
  ImagePacketHeader header{{'T', 'L', 'I', 'M'},
                           1,
                           imageType,
                           width,
                           height,
                           sequence,
                           static_cast<uint32_t>(size)};
  const size_t headerBytes = Serial.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
  const size_t payloadBytes = Serial.write(data, size);
  Serial.flush();
  return headerBytes == sizeof(header) && payloadBytes == size;
}

size_t countJpegBytes(void* arg, size_t index, const void* data, size_t len) {
  (void)index;
  (void)data;
  auto* context = static_cast<JpegCountContext*>(arg);
  context->totalBytes += len;
  return len;
}

size_t writeJpegBytesToSerial(void* arg, size_t index, const void* data, size_t len) {
  (void)index;
  auto* context = static_cast<JpegSerialWriteContext*>(arg);
  const size_t written = Serial.write(static_cast<const uint8_t*>(data), len);
  context->totalBytes += written;
  return written;
}

std::string buildBoundingBoxInfo(const camera_fb_t* fb, const TrafficLightState& state,
                                 uint32_t imageSequence) {
  const int boxWidth = std::max(0, state.red.max_x - state.red.min_x + 1);
  const int boxHeight = std::max(0, state.red.max_y - state.red.min_y + 1);
  const int centerX = std::clamp(state.red.center_x, 0, static_cast<int>(fb->width) - 1);
  const int centerY = std::clamp(state.red.center_y, 0, static_cast<int>(fb->height) - 1);
  const int redLightBoxCenterX = std::clamp(centerX - state.red.min_x, 0, std::max(0, boxWidth - 1));
  const int redLightBoxCenterY = std::clamp(centerY - state.red.min_y, 0, std::max(0, boxHeight - 1));
  int centerR = 0;
  int centerG = 0;
  int centerB = 0;
  unpackRgb565(fb->buf, (centerY * static_cast<int>(fb->width) + centerX) * 2, centerR, centerG, centerB);
  const int centerLuma = computeLuma(centerR, centerG, centerB);
  std::string text;
  text.reserve(256);
  text += "sequence=" + std::to_string(imageSequence) + "\n";
  text += "image_type=red_frame\n";
  text += "frame_width=" + std::to_string(fb->width) + "\n";
  text += "frame_height=" + std::to_string(fb->height) + "\n";
  text += "red_found=" + std::to_string(state.redFound ? 1 : 0) + "\n";
  text += "red_found_by_ei=" + std::to_string(state.redFoundByEi ? 1 : 0) + "\n";
  text += "red_locked=" + std::to_string(state.redLocked ? 1 : 0) + "\n";
  text += "red_on=" + std::to_string(state.redOn ? 1 : 0) + "\n";
  text += "bbox_x_min=" + std::to_string(state.red.min_x) + "\n";
  text += "bbox_y_min=" + std::to_string(state.red.min_y) + "\n";
  text += "bbox_x_max=" + std::to_string(state.red.max_x) + "\n";
  text += "bbox_y_max=" + std::to_string(state.red.max_y) + "\n";
  text += "bbox_width=" + std::to_string(boxWidth) + "\n";
  text += "bbox_height=" + std::to_string(boxHeight) + "\n";
  text += "center_x=" + std::to_string(state.red.center_x) + "\n";
  text += "center_y=" + std::to_string(state.red.center_y) + "\n";
  text += "red_light_box_center_x=" + std::to_string(redLightBoxCenterX) + "\n";
  text += "red_light_box_center_y=" + std::to_string(redLightBoxCenterY) + "\n";
  text += "center_pixel_r=" + std::to_string(centerR) + "\n";
  text += "center_pixel_g=" + std::to_string(centerG) + "\n";
  text += "center_pixel_b=" + std::to_string(centerB) + "\n";
  text += "center_pixel_luma=" + std::to_string(centerLuma) + "\n";
  text += "red_light_box_center_pixel_r=" + std::to_string(centerR) + "\n";
  text += "red_light_box_center_pixel_g=" + std::to_string(centerG) + "\n";
  text += "red_light_box_center_pixel_b=" + std::to_string(centerB) + "\n";
  text += "red_light_box_center_pixel_luma=" + std::to_string(centerLuma) + "\n";
  text += "ei_confidence=" + std::to_string(state.eiConfidence) + "\n";
  text += "ei_crop_x=" + std::to_string(state.eiCropX) + "\n";
  text += "ei_crop_y=" + std::to_string(state.eiCropY) + "\n";
  text += "ei_crop_width=" + std::to_string(state.eiCropWidth) + "\n";
  text += "ei_crop_height=" + std::to_string(state.eiCropHeight) + "\n";
  text += "ei_scale=" + std::to_string(state.eiScale) + "\n";
  text += "ei_resized_width=" + std::to_string(state.eiResizedWidth) + "\n";
  text += "ei_resized_height=" + std::to_string(state.eiResizedHeight) + "\n";
  text += "ei_input_crop_x=" + std::to_string(state.eiInputCropX) + "\n";
  text += "ei_input_crop_y=" + std::to_string(state.eiInputCropY) + "\n";
  text += "ei_input_width=" + std::to_string(EI_CLASSIFIER_INPUT_WIDTH) + "\n";
  text += "ei_input_height=" + std::to_string(EI_CLASSIFIER_INPUT_HEIGHT) + "\n";
  text += "pixel_count=" + std::to_string(state.red.pixel_count) + "\n";
  text += "boxes_count=" + std::to_string(state.serializedBoxCount) + "\n";
  for (size_t i = 0; i < state.serializedBoxCount; ++i) {
    const auto& box = state.serializedBoxes[i];
    text += "box_" + std::to_string(i) + "_label=" + std::string(box.label) + "\n";
    text += "box_" + std::to_string(i) + "_raw_x=" + std::to_string(box.rawX) + "\n";
    text += "box_" + std::to_string(i) + "_raw_y=" + std::to_string(box.rawY) + "\n";
    text += "box_" + std::to_string(i) + "_raw_width=" + std::to_string(box.rawWidth) + "\n";
    text += "box_" + std::to_string(i) + "_raw_height=" + std::to_string(box.rawHeight) + "\n";
    text += "box_" + std::to_string(i) + "_x_min=" + std::to_string(box.xMin) + "\n";
    text += "box_" + std::to_string(i) + "_y_min=" + std::to_string(box.yMin) + "\n";
    text += "box_" + std::to_string(i) + "_x_max=" + std::to_string(box.xMax) + "\n";
    text += "box_" + std::to_string(i) + "_y_max=" + std::to_string(box.yMax) + "\n";
    text += "box_" + std::to_string(i) + "_cluster_center_projected_x=" +
            std::to_string(box.clusterCenterX) + "\n";
    text += "box_" + std::to_string(i) + "_cluster_center_projected_y=" +
            std::to_string(box.clusterCenterY) + "\n";
    text += "box_" + std::to_string(i) + "_width=" +
            std::to_string(std::max(0, box.xMax - box.xMin + 1)) + "\n";
    text += "box_" + std::to_string(i) + "_height=" +
            std::to_string(std::max(0, box.yMax - box.yMin + 1)) + "\n";
    text += "box_" + std::to_string(i) + "_confidence=" + std::to_string(box.confidence) + "\n";
  }
  return text;
}

void sendEiModelInputSnapshot(const TrafficLightState& state, uint32_t sequence) {
  if (!sendImages) {
    return;
  }
  (void)state;
  if (gEiInputBuffer == nullptr) {
    return;
  }
  const size_t payloadSize = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3;
  const bool wrote =
      writeBinaryPacket(kImageTypeEiModelInput, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT,
                        sequence, gEiInputBuffer, payloadSize);
  if (wrote) {
    Serial.printf("SNAPSHOT ei_model_input sent seq=%lu bytes=%lu\n",
                  static_cast<unsigned long>(sequence), static_cast<unsigned long>(payloadSize));
  } else {
    Serial.printf("SNAPSHOT ei_model_input send failed seq=%lu\n",
                  static_cast<unsigned long>(sequence));
  }
}

void sendRawEiOutput(const TrafficLightState& state, uint32_t sequence) {
  if (!sendImages) {
    return;
  }
  if (state.rawEiOutput.empty()) {
    return;
  }
  const bool wrote =
      writeTextPacket(kImageTypeRawEiOutput, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT,
                      sequence, state.rawEiOutput);
  if (wrote) {
    Serial.printf("SNAPSHOT raw_ei_output sent seq=%lu bytes=%lu\n",
                  static_cast<unsigned long>(sequence),
                  static_cast<unsigned long>(state.rawEiOutput.size()));
  } else {
    Serial.printf("SNAPSHOT raw_ei_output send failed seq=%lu\n",
                  static_cast<unsigned long>(sequence));
  }
}

void sendEiCropRoiSnapshot(const camera_fb_t* fb, const TrafficLightState& state, uint32_t sequence) {
  if (!sendImages) {
    return;
  }
  if (!state.hasEiCrop || state.eiCropWidth <= 0 || state.eiCropHeight <= 0) {
    return;
  }

  const size_t cropBytes =
      static_cast<size_t>(state.eiCropWidth) * static_cast<size_t>(state.eiCropHeight) * 2;
  uint8_t* cropBuffer = static_cast<uint8_t*>(malloc(cropBytes));
  if (cropBuffer == nullptr) {
    Serial.println("SNAPSHOT ei_crop_roi alloc failed");
    return;
  }

  const size_t srcStride = static_cast<size_t>(fb->width) * 2;
  const size_t rowBytes = static_cast<size_t>(state.eiCropWidth) * 2;
  const uint8_t* src = fb->buf + (static_cast<size_t>(state.eiCropY) * static_cast<size_t>(fb->width) +
                                  static_cast<size_t>(state.eiCropX)) *
                                     2;
  for (int y = 0; y < state.eiCropHeight; ++y) {
    std::memcpy(cropBuffer + static_cast<size_t>(y) * rowBytes,
                src + static_cast<size_t>(y) * srcStride, rowBytes);
  }

  uint8_t* jpegData = nullptr;
  size_t jpegSize = 0;
  if (!fmt2jpg(cropBuffer, cropBytes, state.eiCropWidth, state.eiCropHeight, PIXFORMAT_RGB565,
               kSnapshotJpegQuality, &jpegData, &jpegSize) ||
      jpegData == nullptr || jpegSize == 0) {
    free(cropBuffer);
    Serial.println("SNAPSHOT ei_crop_roi encode failed");
    return;
  }
  free(cropBuffer);

  const bool wrote = writeBinaryPacket(kImageTypeEiCropRoi, static_cast<uint16_t>(state.eiCropWidth),
                                       static_cast<uint16_t>(state.eiCropHeight), sequence,
                                       jpegData, jpegSize);
  free(jpegData);

  if (wrote) {
    Serial.printf("SNAPSHOT ei_crop_roi sent seq=%lu bytes=%lu\n",
                  static_cast<unsigned long>(sequence), static_cast<unsigned long>(jpegSize));
  } else {
    Serial.printf("SNAPSHOT ei_crop_roi send failed seq=%lu\n",
                  static_cast<unsigned long>(sequence));
  }
}

void sendGreenWatchRedBoxSnapshot(const camera_fb_t* fb, const GreenWatchState& watch, uint32_t sequence) {
  if (!sendImages) {
    return;
  }
  if (!watch.active) {
    return;
  }

  const int sourceWidth = static_cast<int>(fb->width);
  const int sourceHeight = static_cast<int>(fb->height);
  const int minX = std::clamp(scaleCoordinate(watch.red.min_x, watch.frameWidth, sourceWidth), 0,
                              sourceWidth - 1);
  const int minY = std::clamp(scaleCoordinate(watch.red.min_y, watch.frameHeight, sourceHeight), 0,
                              sourceHeight - 1);
  const int maxX = std::clamp(scaleCoordinate(watch.red.max_x, watch.frameWidth, sourceWidth), minX,
                              sourceWidth - 1);
  const int maxY = std::clamp(scaleCoordinate(watch.red.max_y, watch.frameHeight, sourceHeight), minY,
                              sourceHeight - 1);
  const int centerX = std::clamp(scaleCoordinate(watch.red.center_x, watch.frameWidth, sourceWidth), minX,
                                 maxX);
  const int centerY = std::clamp(scaleCoordinate(watch.red.center_y, watch.frameHeight, sourceHeight), minY,
                                 maxY);
  const int cropWidth = std::max(1, maxX - minX + 1);
  const int cropHeight = std::max(1, maxY - minY + 1);
  const int localCenterX = std::clamp(centerX - minX, 0, cropWidth - 1);
  const int localCenterY = std::clamp(centerY - minY, 0, cropHeight - 1);

  const size_t cropBytes =
      static_cast<size_t>(cropWidth) * static_cast<size_t>(cropHeight) * 2;
  uint8_t* cropBuffer = static_cast<uint8_t*>(malloc(cropBytes));
  if (cropBuffer == nullptr) {
    Serial.println("SNAPSHOT red_light_box alloc failed");
    return;
  }

  const size_t srcStride = static_cast<size_t>(fb->width) * 2;
  const size_t rowBytes = static_cast<size_t>(cropWidth) * 2;
  const uint8_t* src =
      fb->buf + (static_cast<size_t>(minY) * static_cast<size_t>(fb->width) + static_cast<size_t>(minX)) * 2;
  for (int y = 0; y < cropHeight; ++y) {
    std::memcpy(cropBuffer + static_cast<size_t>(y) * rowBytes,
                src + static_cast<size_t>(y) * srcStride, rowBytes);
  }
  const size_t centerIndex =
      (static_cast<size_t>(localCenterY) * static_cast<size_t>(cropWidth) +
       static_cast<size_t>(localCenterX)) * 2;
  cropBuffer[centerIndex] = 0;
  cropBuffer[centerIndex + 1] = 0;

  uint8_t* jpegData = nullptr;
  size_t jpegSize = 0;
  if (!fmt2jpg(cropBuffer, cropBytes, cropWidth, cropHeight, PIXFORMAT_RGB565,
               kSnapshotJpegQuality, &jpegData, &jpegSize) ||
      jpegData == nullptr || jpegSize == 0) {
    free(cropBuffer);
    Serial.println("SNAPSHOT red_light_box encode failed");
    return;
  }
  free(cropBuffer);

  const bool wrote = writeBinaryPacket(kImageTypeRedLightBox, static_cast<uint16_t>(cropWidth),
                                       static_cast<uint16_t>(cropHeight), sequence,
                                       jpegData, jpegSize);
  free(jpegData);

  if (wrote) {
    Serial.printf("SNAPSHOT red_light_box sent seq=%lu box=(%d,%d,%d,%d) center=(%d,%d) local_center=(%d,%d) bytes=%lu\n",
                  static_cast<unsigned long>(sequence), minX, minY, maxX, maxY,
                  centerX, centerY, localCenterX, localCenterY,
                  static_cast<unsigned long>(jpegSize));
  } else {
    Serial.printf("SNAPSHOT red_light_box send failed seq=%lu\n",
                  static_cast<unsigned long>(sequence));
  }
}

void sendFullFrameSnapshot(const camera_fb_t* fb, const TrafficLightState* state,
                           ImageType imageType = kImageTypeRedFrame) {
  if (!sendImages) {
    Serial.println("SNAPSHOT image sending disabled");
    return;
  }
  Serial.printf("SNAPSHOT begin type=%u frame=%ux%u len=%u quality=%u\n", imageType, fb->width,
                fb->height, static_cast<unsigned>(fb->len), kSnapshotJpegQuality);
  JpegCountContext countContext{};
  if (!fmt2jpg_cb(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, kSnapshotJpegQuality,
                  countJpegBytes, &countContext) ||
      countContext.totalBytes == 0) {
    Serial.println("SNAPSHOT red_frame encode failed");
    return;
  }
  Serial.printf("SNAPSHOT counted bytes=%lu\n", static_cast<unsigned long>(countContext.totalBytes));

  const uint32_t sequence = gPacketSequence++;
  ImagePacketHeader header{{'T', 'L', 'I', 'M'},
                           1,
                           imageType,
                           fb->width,
                           fb->height,
                           sequence,
                           static_cast<uint32_t>(countContext.totalBytes)};
  const size_t headerBytes = Serial.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
  if (headerBytes != sizeof(header)) {
    Serial.println("SNAPSHOT red_frame header write failed");
    return;
  }

  JpegSerialWriteContext writeContext{};
  if (!fmt2jpg_cb(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, kSnapshotJpegQuality,
                  writeJpegBytesToSerial, &writeContext) ||
      writeContext.totalBytes != countContext.totalBytes) {
    Serial.println("SNAPSHOT red_frame serial write failed");
    return;
  }

  Serial.flush();
  Serial.printf("SNAPSHOT sent type=%u bytes=%lu seq=%lu\n", imageType,
                static_cast<unsigned long>(writeContext.totalBytes),
                static_cast<unsigned long>(sequence));

  if ((imageType == kImageTypeRedFrame || imageType == kImageTypeRedFrameEi) &&
      state != nullptr && state->redFound) {
    const std::string bboxInfo = buildBoundingBoxInfo(fb, *state, sequence);
    if (writeTextPacket(kImageTypeRedFrameBoxes, fb->width, fb->height, sequence, bboxInfo)) {
      Serial.printf("SNAPSHOT bbox sent seq=%lu bytes=%u\n", static_cast<unsigned long>(sequence),
                    static_cast<unsigned>(bboxInfo.size()));
    } else {
      Serial.printf("SNAPSHOT bbox send failed seq=%lu\n", static_cast<unsigned long>(sequence));
    }
  }

  if (state != nullptr && state->hasEiCrop) {
    sendRawEiOutput(*state, sequence);
    sendEiModelInputSnapshot(*state, sequence);
    sendEiCropRoiSnapshot(fb, *state, sequence);
  }
}

void sendTrafficLightSnapshots(const camera_fb_t* fb, const TrafficLightState& state,
                               uint32_t nowMs) {
  if (!sendImages) {
    Serial.println("SNAPSHOT image sending disabled");
    return;
  }

  Serial.println("SNAPSHOT periodic send");
  sendFullFrameSnapshot(fb, &state,
                        state.redFound ? (state.redFoundByEi ? kImageTypeRedFrameEi
                                                             : kImageTypeRedFrame)
                                       : kImageTypePeriodicFrame);

  const bool triggered = state.redFound && shouldSendSnapshots(state, nowMs);
  if (triggered) {
    sendBleFrameSnapshot(fb, "red_light_detected", nowMs);
  }
}

}  // namespace

// Default loopTask stack (8 KB) is too small for run_classifier on 1280x1024.
SET_LOOP_TASK_STACK_SIZE(32 * 1024);

void setup() {
  Serial.begin(kSerialBaudRate);
  delay(1500);
  gCameraMutex = xSemaphoreCreateMutex();
  Serial.println();
  Serial.println("esp32_serial_camera_sender boot");
  Serial.printf("Chip model: %s\n", ESP.getChipModel());
  Serial.printf("Chip revision: %d\n", ESP.getChipRevision());
  Serial.printf("firmware=%s\n", kFirmwareVersion);
  Serial.printf("Arduino ESP32 core version: %s\n", ESP.getSdkVersion());
  Serial.println("BMA400 hello world");
  Serial.printf("BMA400 SPI pins: SCK=%d MISO=%d MOSI=%d CS=%d\n",
                kBma400SpiSckPin, kBma400SpiMisoPin, kBma400SpiMosiPin, kBma400SpiCsPin);
  Serial.println("BMA400 wiring: SCL->SCK, SDA->MOSI, SDO->MISO, CSB->CS");
  gBma400Ready = setupBma400();
  Serial.printf("BMA400 ready=%d\n", gBma400Ready ? 1 : 0);

#if defined(BOARD_HAS_PSRAM)
  Serial.println("BOARD_HAS_PSRAM defined");
#else
  Serial.println("BOARD_HAS_PSRAM not defined");
#endif
  Serial.println("OV5640 AF library: required");
  const bool psramFoundBeforeInit = psramFound();
  Serial.printf("psramFound(before init)=%s\n", psramFoundBeforeInit ? "true" : "false");
  const bool psramInitResult = psramInit();
  const bool psramFoundAfterInit = psramFound();
  Serial.printf("psramInit()=%s\n", psramInitResult ? "true" : "false");
  Serial.printf("psramFound(after init)=%s\n", psramFoundAfterInit ? "true" : "false");
  Serial.printf("ESP.getPsramSize()=%lu\n", static_cast<unsigned long>(ESP.getPsramSize()));
  if (!psramFoundAfterInit) {
    Serial.println("PSRAM not detected. Check board selection and PSRAM setting in Arduino IDE.");
  }
  setupCamera();
  Serial.println("camera init complete");
  setupAutofocus();
  setupBleServer();
  // startDebugMjpegServer();
  if (kEnableTrafficLightDetector) {
    Serial.println("starting on-device traffic light detector");
  }
  delay(250);
  setCurrentBleStatus("Booted", "Camera initialized and awaiting BLE connection.", false, false,
                      millis(), true);
}

void loop() {
  ++gLoopCounter;
  const uint32_t loopNow = millis();
  logBma400MotionData(loopNow);
  if (kEnableBleServer && loopNow - gLastBleHeartbeatLogMs >= 10000) {
    gLastBleHeartbeatLogMs = loopNow;
    Serial.printf("BLE heartbeat connected=%d latest_image_requested=%d paused=%d\n",
                  gBleClientConnected ? 1 : 0, gBleLatestImageRequested ? 1 : 0,
                  gPaused ? 1 : 0);
  }

  if (gPaused) {
    gBleLatestImageRequested = false;
    gBleSendWindowRequested = false;
    notifyCurrentBleStatusOnLoop(loopNow);
    delay(1);
    return;
  }

  const bool shouldRunGreenWatch =
      kEnableTrafficLightDetector && gGreenWatch.active &&
      (loopNow - gGreenWatch.lastCheckAtMs >= kGreenWatchCheckIntervalMs);
  const bool shouldRunDetector =
      kEnableTrafficLightDetector && !gGreenWatch.active &&
      (loopNow - gLastDetectorRunMs >= kDetectorIntervalMs);
  const bool shouldCaptureFrame = shouldRunGreenWatch || shouldRunDetector || gBleLatestImageRequested;
  if (!shouldCaptureFrame) {
    if (gBleSendWindowRequested) {
      gBleSendWindowRequested = false;
      if (gBleActiveImageData != nullptr && gBleActiveImageSize > 0) {
        Serial.printf("BLE send window from sequence=%u active_bytes=%u seq=%lu window_chunks=%u\n",
                      static_cast<unsigned>(gBleSendWindowSequence),
                      static_cast<unsigned>(gBleActiveImageSize),
                      static_cast<unsigned long>(gBleActiveImageSequence),
                      bleImageWindowChunkCount(gBleActiveImageSize));
        notifyBleImageChunks(gBleActiveImageData, gBleActiveImageSize, gBleSendWindowSequence);
      } else {
        Serial.printf("BLE send window skipped no active image sequence=%u\n",
                      static_cast<unsigned>(gBleSendWindowSequence));
      }
    }
    notifyCurrentBleStatusOnLoop(loopNow);
    delay(1);
    return;
  }

  camera_fb_t* fb = captureFrameLocked();
  if (!fb) {
    delay(10);
    return;
  }

  bool startedGreenWatchThisFrame = false;
  if (kEnableTrafficLightDetector) {
    const uint32_t now = millis();
    if (gGreenWatch.active) {
      if (shouldRunGreenWatch) {
        Serial.println("GREEN_WATCH active skip_ei=1");
        const GreenWatchUpdateResult greenWatchResult = updateGreenWatch(fb, now);
        if (sendImages) {
          sendFullFrameSnapshot(fb, nullptr, kImageTypeGreenLightSearch);
          sendGreenWatchRedBoxSnapshot(fb, gGreenWatch, gPacketSequence++);
        } else {
          Serial.println("SNAPSHOT image sending disabled");
        }
        if (greenWatchResult == GreenWatchUpdateResult::kRedOffDetected) {
          setCurrentBleStatus("Green observed", "Red lamp cleared and the signal changed.", false,
                              true, now, true);
          sendBleFrameSnapshot(fb, "green_light_detected", now);
          sendFullFrameSnapshot(fb, nullptr, kImageTypeGreenLight);
        } else if (greenWatchResult == GreenWatchUpdateResult::kSceneChanged) {
          setCurrentBleStatus("Scene changed", "Tracking target moved or the scene changed.", false,
                              false, now, true);
          sendFullFrameSnapshot(fb, nullptr, kImageTypeSceneChanged);
        }
      }
    } else if (shouldRunDetector) {
      gLastDetectorRunMs = now;
      adjustExposureForSun(fb);
      // waitForAutofocusFocused();
      const TrafficLightState state = analyzeTrafficLight(fb);
      reportTrafficLightState(state, now);
      if (state.redFound) {
        sendBleFrameSnapshot(fb, "red_light_detected", now);
      }
      sendTrafficLightSnapshots(fb, state, now);
      if (state.redFoundByEi && state.redFound) {
        startGreenWatch(fb, state.red, now);
        startedGreenWatchThisFrame = true;
      }
    }
  }

  if (gBleLatestImageRequested) {
    gBleLatestImageRequested = false;
    setCurrentBleStatus("Sending image", "Sending the latest requested frame over BLE.", false,
                        false, millis(), true);
    sendBleFrameSnapshot(fb, "manual_request", millis());
  }

  if (gBleSendWindowRequested) {
    gBleSendWindowRequested = false;
    if (gBleActiveImageData != nullptr && gBleActiveImageSize > 0) {
      Serial.printf("BLE send window from sequence=%u active_bytes=%u seq=%lu window_chunks=%u\n",
                    static_cast<unsigned>(gBleSendWindowSequence),
                    static_cast<unsigned>(gBleActiveImageSize),
                    static_cast<unsigned long>(gBleActiveImageSequence),
                    bleImageWindowChunkCount(gBleActiveImageSize));
      notifyBleImageChunks(gBleActiveImageData, gBleActiveImageSize, gBleSendWindowSequence);
    } else {
      Serial.printf("BLE send window skipped no active image sequence=%u\n",
                    static_cast<unsigned>(gBleSendWindowSequence));
    }
  }

  notifyCurrentBleStatusOnLoop(loopNow);
  releaseFrameLocked(fb);
  if (startedGreenWatchThisFrame) {
    enterGreenWatchLowResolutionMode(loopNow);
  } else if (!gGreenWatch.active && gGreenWatchLowResolutionMode) {
    exitGreenWatchLowResolutionMode();
  }
  delay(1);
}
