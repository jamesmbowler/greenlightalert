#pragma once

#include <Arduino.h>

#include "esp_camera.h"
#include "shared_detection_config.h"

enum class BleImageSource : uint8_t {
  FullFrame = 0,
  EdgeImpulseInput = 1,
};

// Conservative defaults for ESP32-S3 camera bring-up.
constexpr uint32_t kSerialBaudRate = 921600;
constexpr bool kEnableTrafficLightDetector = true;
constexpr bool kEnableEdgeImpulseFirstLine = true;
constexpr bool kSensorHorizontalMirror = true;
constexpr bool kSensorVerticalFlip = false;
constexpr bool kEnableOv5640RawWindow = true;
constexpr bool kUseOv5640DirectWindowRegisters = true;
constexpr bool kDumpOv5640PresetRegisters = true;
constexpr int kOv5640WindowWidthPixels = 1280;
constexpr int kOv5640WindowHeightPixels = 1024;
constexpr float kOv5640WindowCenterXFraction = 0.50f;
constexpr float kOv5640WindowCenterYFraction = 0.50f;
constexpr bool kOv5640WindowLockPresetAspect = false;
constexpr bool kOv5640WindowUseCroppedOutput = true;
constexpr int kOv5640WindowOutputWidth = 0;   // >0 overrides output width
constexpr int kOv5640WindowOutputHeight = 0;  // >0 overrides output height
constexpr bool kEnableOv5640SensorDigitalZoom = false;
constexpr float kOv5640SensorDigitalZoomFactor = 1.5f;
constexpr bool useBackupAlgo = false;
constexpr float kEdgeImpulseMinConfidence = sharedcfg::kEdgeImpulseMinConfidence;
constexpr float kEiCropWidthFraction = 0.50f;
constexpr float kEiCropHeightFraction = 0.50f;
constexpr float kEiCropCenterXFraction = 0.50f;
constexpr float kEiCropCenterYFraction = 0.15f;
constexpr bool sendImages = false;
constexpr bool sendImagesBle = true;
constexpr bool kEnableBleServer = true;
constexpr char kBleDeviceName[] = "GreenLightAlert ESP32";
constexpr uint16_t kBlePreferredMtu = 517;
constexpr uint16_t kBleChunkSequenceHeaderSize = 2;
constexpr uint16_t kBleImageChunkPayloadSize = 180;
constexpr uint16_t kBleImageChunkSize = kBleImageChunkPayloadSize + kBleChunkSequenceHeaderSize;
constexpr uint8_t kBleImageWindowParts = 4;
constexpr uint32_t kBleChunkDelayMs = 12;
constexpr uint32_t kBleMetadataLeadDelayMs = 60;
constexpr BleImageSource kBleImageSource = BleImageSource::EdgeImpulseInput;
constexpr uint32_t kBleStatusHeartbeatMs = 15000;
constexpr bool kEnableSunExposureCompensation = false;
constexpr int kSunSampleStride = 8;
constexpr int kSunBrightLumaThreshold = 240;
constexpr float kSunBrightFractionThreshold = 0.20f;
constexpr float kSunRecoverFractionThreshold = 0.05f;
constexpr int kSunAeLevel = -2;
constexpr framesize_t kFrameSizeWithPsram = FRAMESIZE_SXGA;   // 1280x1024 so the HAL can accept larger custom OV5640 window outputs
constexpr framesize_t kFrameSizeWithoutPsram = FRAMESIZE_QVGA;
constexpr int kJpegQualityWithPsram = 14;
constexpr int kJpegQualityWithoutPsram = 20;
constexpr int kFrameBufferCountWithPsram = 1;
constexpr int kFrameBufferCountWithoutPsram = 1;
constexpr bool kEnableOv5640Autofocus = true;
constexpr bool kWaitForOv5640FocusedBeforeCapture = false;
constexpr uint32_t kAutofocusSettleDelayMs = 1500;
constexpr uint32_t kAutofocusPollIntervalMs = 20;
constexpr uint32_t kAutofocusFocusTimeoutMs = 500;
// AE level: -2 (darker/backlit scenes) to +2 (brighter). 0 = sensor default.
constexpr int kAeLevel = 0;
constexpr uint32_t kDetectorIntervalMs = 1000;
constexpr int kScanStride = 3;
constexpr int kGridCols = 8;
constexpr int kGridRows = 6;
constexpr int kMinRedComponent = sharedcfg::kMinRedComponent;
constexpr int kMinRedExcessOverGreen = sharedcfg::kMinRedExcessOverGreen;
constexpr int kMinRedExcessOverBlue = sharedcfg::kMinRedExcessOverBlue;
constexpr int kMinRedPixelsForDetection = sharedcfg::kMinRedPixelsForDetection;
constexpr float kYellowYOffsetMultiplier = 2.5f;
constexpr float kGreenYOffsetMultiplier = 5.0f;
constexpr int kMinGreenRoiRadius = 8;
constexpr int kSearchMinY = 0;
constexpr float kSearchMaxYFraction = 0.70f;
constexpr int kMinLampRadius = 2;
constexpr int kMaxLampRadius = 20;
constexpr float kMaxLampAspectRatio = 1.45f;
constexpr int kDarkComponentMax = 80;
constexpr float kMinDarkFractionAtExpectedLamps = 0.18f;
constexpr uint32_t kSnapshotCooldownMs = 3000;
constexpr int kStableFramesRequired = 1;
constexpr int kRedCenterMoveTolerance = 12;
constexpr int kRedLockDarkFramesToRelease = 4;
constexpr uint32_t kGreenWatchCheckIntervalMs = sharedcfg::kGreenWatchCheckIntervalMs;
constexpr uint32_t kGreenWatchTimeoutMs = sharedcfg::kGreenWatchTimeoutMs;
constexpr uint32_t kGreenWatchSceneChangeGraceMs = sharedcfg::kGreenWatchSceneChangeGraceMs;
constexpr int kGreenWatchDarkLumaThreshold = sharedcfg::kGreenWatchDarkLumaThreshold;
constexpr int kGreenWatchCenterDarkLumaThreshold = sharedcfg::kGreenWatchCenterDarkLumaThreshold;
constexpr int kGreenWatchCenterDarkDelta = sharedcfg::kGreenWatchCenterDarkDelta;
constexpr int kGreenWatchCenterSearchBoxSize = sharedcfg::kGreenWatchCenterSearchBoxSize;
constexpr int kGreenWatchSimilarRedDelta = sharedcfg::kGreenWatchSimilarRedDelta;
constexpr int kGreenWatchSimilarGreenDelta = sharedcfg::kGreenWatchSimilarGreenDelta;
constexpr int kGreenWatchSimilarBlueDelta = sharedcfg::kGreenWatchSimilarBlueDelta;
constexpr float kGreenWatchSceneDiffThreshold = sharedcfg::kGreenWatchSceneDiffThreshold;
constexpr float kGreenWatchPatchScale = sharedcfg::kGreenWatchPatchScale;
constexpr int kEiCandidateMinRedClusterPixels = sharedcfg::kEiCandidateMinRedClusterPixels;
constexpr float kEiCandidateMaxClusterAspectRatio = sharedcfg::kEiCandidateMaxClusterAspectRatio;
constexpr float kEiCandidateMinClusterFillFraction = sharedcfg::kEiCandidateMinClusterFillFraction;
constexpr float kEiBoundingBoxPaddingFraction = sharedcfg::kEiBoundingBoxPaddingFraction;
constexpr int kEiClusterEdgeExpansionSteps = sharedcfg::kEiClusterEdgeExpansionSteps;
constexpr float kEiClusterEdgeExpansionFraction = sharedcfg::kEiClusterEdgeExpansionFraction;
constexpr int kEiWarmBrightMinRedComponent = sharedcfg::kEiWarmBrightMinRedComponent;
constexpr int kEiWarmBrightMinGreenComponent = sharedcfg::kEiWarmBrightMinGreenComponent;
constexpr int kEiWarmBrightMaxBlueComponent = sharedcfg::kEiWarmBrightMaxBlueComponent;
constexpr int kEiWarmBrightMinBlueGap = sharedcfg::kEiWarmBrightMinBlueGap;
constexpr uint8_t kSnapshotJpegQuality = 45;
constexpr uint8_t kBleSnapshotJpegQuality = 60;

inline camera_config_t makeCameraConfig() {
  const bool hasPsram = psramFound();

  camera_config_t config{};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = kEnableTrafficLightDetector ? PIXFORMAT_RGB565 : PIXFORMAT_JPEG;
  config.frame_size = hasPsram ? kFrameSizeWithPsram : kFrameSizeWithoutPsram;
  config.jpeg_quality = hasPsram ? kJpegQualityWithPsram : kJpegQualityWithoutPsram;
  config.fb_count = hasPsram ? kFrameBufferCountWithPsram : kFrameBufferCountWithoutPsram;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = hasPsram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  return config;
}
