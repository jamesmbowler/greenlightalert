#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "arduino/esp32_serial_camera_sender/red_light_box_shared.h"
#include "arduino/esp32_serial_camera_sender/shared_detection_config.h"
#include "jbowler-project-1_inferencing.h"

namespace fs = std::filesystem;

namespace {

constexpr int kPatchSignatureSize = 16;

struct ImageFrame {
  std::string name;
  uint64_t timestampMs = 0;
  int width = 0;
  int height = 0;
  cv::Mat bgr;
  std::vector<uint8_t> rgb888;
  std::vector<uint8_t> rgb565;
};

struct DetectionBox {
  std::string label;
  int rawX = 0;
  int rawY = 0;
  int rawWidth = 0;
  int rawHeight = 0;
  int searchX1 = 0;
  int searchY1 = 0;
  int searchX2 = 0;
  int searchY2 = 0;
  int adjustedX1 = 0;
  int adjustedY1 = 0;
  int adjustedX2 = 0;
  int adjustedY2 = 0;
  int centerX = 0;
  int centerY = 0;
  float confidence = 0.0f;
  bool accepted = false;
  redbox::RedClusterCheckResult cluster{};
};

struct DetectionResult {
  bool redFound = false;
  float confidence = 0.0f;
  uint32_t inferenceDurationMs = 0;
  uint32_t boundingBoxCount = 0;
  DetectionBox bestBox{};
  std::vector<DetectionBox> confidentBoxes;
  std::string rawEiOutput;
};

struct GreenWatchState {
  bool active = false;
  uint64_t startedAtMs = 0;
  uint64_t lastCheckAtMs = 0;
  uint64_t sceneCheckReadyAtMs = 0;
  int frameWidth = 0;
  int frameHeight = 0;
  DetectionBox red{};
  int baselineCenterLuma = 0;
  int baselineCenterR = 0;
  int baselineCenterG = 0;
  int baselineCenterB = 0;
  int coreX = 0;
  int coreY = 0;
  int coreWidth = 0;
  int coreHeight = 0;
  int patchX = 0;
  int patchY = 0;
  int patchWidth = 0;
  int patchHeight = 0;
  uint8_t patchSignature[kPatchSignatureSize * kPatchSignatureSize]{};
};

enum class GreenWatchUpdateResult {
  None,
  RedOffDetected,
  SceneChanged,
};

struct SequenceResult {
  std::string fileName;
  uint64_t timestampMs = 0;
  DetectionResult detection;
  bool greenWatchActive = false;
  GreenWatchUpdateResult greenWatchResult = GreenWatchUpdateResult::None;
};

const std::vector<uint8_t>* gSignalRgb888 = nullptr;

int computeLuma(int r, int g, int b) {
  return (77 * r + 150 * g + 29 * b) >> 8;
}

void unpackRgb565(const uint8_t* data, int width, int x, int y, int& r, int& g, int& b) {
  const size_t index = static_cast<size_t>((y * width + x) * 2);
  const uint16_t pixel =
      static_cast<uint16_t>((static_cast<uint16_t>(data[index]) << 8) | data[index + 1]);
  r = ((pixel >> 11) & 0x1F) * 255 / 31;
  g = ((pixel >> 5) & 0x3F) * 255 / 63;
  b = (pixel & 0x1F) * 255 / 31;
}

bool isRedPixel(int r, int g, int b) {
  return r >= sharedcfg::kMinRedComponent &&
         (r - g) >= sharedcfg::kMinRedExcessOverGreen &&
         (r - b) >= sharedcfg::kMinRedExcessOverBlue;
}

void padBoundingBox(int& x1, int& y1, int& x2, int& y2, int frameWidth, int frameHeight) {
  const int width = std::max(1, x2 - x1 + 1);
  const int height = std::max(1, y2 - y1 + 1);
  const int padX =
      std::max(1, static_cast<int>(std::ceil(width * sharedcfg::kEiBoundingBoxPaddingFraction)));
  const int padY =
      std::max(1, static_cast<int>(std::ceil(height * sharedcfg::kEiBoundingBoxPaddingFraction)));
  x1 = std::max(0, x1 - padX);
  y1 = std::max(0, y1 - padY);
  x2 = std::min(frameWidth - 1, x2 + padX);
  y2 = std::min(frameHeight - 1, y2 + padY);
}

bool clusterTouchesBoxEdge(const redbox::RedClusterCheckResult& result, int boxWidth, int boxHeight) {
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
      std::max(1, static_cast<int>(std::ceil(width * sharedcfg::kEiClusterEdgeExpansionFraction)));
  const int expandY =
      std::max(1, static_cast<int>(std::ceil(height * sharedcfg::kEiClusterEdgeExpansionFraction)));
  x1 = std::max(0, x1 - expandX);
  y1 = std::max(0, y1 - expandY);
  x2 = std::min(frameWidth - 1, x2 + expandX);
  y2 = std::min(frameHeight - 1, y2 + expandY);
}

uint64_t parseTimestampMs(const fs::path& path) {
  const std::string stem = path.stem().string();
  size_t end = stem.size();
  while (end > 0 && std::isdigit(static_cast<unsigned char>(stem[end - 1]))) {
    --end;
  }
  if (end == stem.size()) {
    return 0;
  }
  return static_cast<uint64_t>(std::stoull(stem.substr(end)));
}

bool isImagePath(const fs::path& path) {
  if (!path.has_extension()) {
    return false;
  }
  std::string ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return ext == ".jpg" || ext == ".jpeg" || ext == ".png";
}

std::vector<uint8_t> convertBgrToRgb888(const cv::Mat& bgr) {
  std::vector<uint8_t> rgb888(static_cast<size_t>(bgr.cols * bgr.rows * 3));
  for (int y = 0; y < bgr.rows; ++y) {
    for (int x = 0; x < bgr.cols; ++x) {
      const cv::Vec3b pixel = bgr.at<cv::Vec3b>(y, x);
      const size_t offset = static_cast<size_t>((y * bgr.cols + x) * 3);
      rgb888[offset + 0] = pixel[2];
      rgb888[offset + 1] = pixel[1];
      rgb888[offset + 2] = pixel[0];
    }
  }
  return rgb888;
}

std::vector<uint8_t> convertRgb888ToRgb565(const std::vector<uint8_t>& rgb888, int width, int height) {
  std::vector<uint8_t> rgb565(static_cast<size_t>(width * height * 2));
  for (int i = 0; i < width * height; ++i) {
    const uint8_t r = rgb888[static_cast<size_t>(i) * 3 + 0];
    const uint8_t g = rgb888[static_cast<size_t>(i) * 3 + 1];
    const uint8_t b = rgb888[static_cast<size_t>(i) * 3 + 2];
    const uint16_t value = static_cast<uint16_t>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
    rgb565[static_cast<size_t>(i) * 2 + 0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    rgb565[static_cast<size_t>(i) * 2 + 1] = static_cast<uint8_t>(value & 0xFF);
  }
  return rgb565;
}

ImageFrame loadFrame(const fs::path& path) {
  cv::Mat bgr = cv::imread(path.string(), cv::IMREAD_COLOR);
  if (bgr.empty()) {
    throw std::runtime_error("Failed to load image: " + path.string());
  }
  if (bgr.cols != EI_CLASSIFIER_INPUT_WIDTH || bgr.rows != EI_CLASSIFIER_INPUT_HEIGHT) {
    cv::resize(bgr, bgr, cv::Size(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT), 0, 0,
               cv::INTER_LINEAR);
  }

  ImageFrame frame;
  frame.name = path.filename().string();
  frame.timestampMs = parseTimestampMs(path);
  frame.width = bgr.cols;
  frame.height = bgr.rows;
  frame.bgr = bgr;
  frame.rgb888 = convertBgrToRgb888(bgr);
  frame.rgb565 = convertRgb888ToRgb565(frame.rgb888, frame.width, frame.height);
  return frame;
}

static int signalGetData(size_t offset, size_t length, float* outPtr) {
  if (gSignalRgb888 == nullptr) {
    return -1;
  }
  const size_t totalPixels = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  if (offset >= totalPixels) {
    return -1;
  }
  if (offset + length > totalPixels) {
    length = totalPixels - offset;
  }
  size_t pixelIndex = offset * 3;
  for (size_t outIndex = 0; outIndex < length; ++outIndex) {
    outPtr[outIndex] = ((*gSignalRgb888)[pixelIndex] << 16) |
                       ((*gSignalRgb888)[pixelIndex + 1] << 8) |
                       (*gSignalRgb888)[pixelIndex + 2];
    pixelIndex += 3;
  }
  return 0;
}

DetectionResult detectRedLight(const ImageFrame& frame) {
  DetectionResult detection;
  gSignalRgb888 = &frame.rgb888;
  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &signalGetData;

  ei_impulse_result_t result = {0};
  const auto start = std::chrono::steady_clock::now();
  const EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
  const auto end = std::chrono::steady_clock::now();
  detection.inferenceDurationMs = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
  detection.boundingBoxCount = result.bounding_boxes_count;
  detection.rawEiOutput += "ei_result=" + std::to_string(static_cast<int>(err)) + "\n";
  detection.rawEiOutput += "inference_duration_ms=" + std::to_string(detection.inferenceDurationMs) + "\n";
  detection.rawEiOutput += "bounding_boxes_count=" + std::to_string(result.bounding_boxes_count) + "\n";
  if (err != EI_IMPULSE_OK) {
    return detection;
  }

  bool found = false;
  float bestScore = 0.0f;
  int bestTopY = std::numeric_limits<int>::max();

  for (uint32_t i = 0; i < result.bounding_boxes_count; ++i) {
    const auto& bb = result.bounding_boxes[i];
    detection.rawEiOutput += "box_" + std::to_string(i) + "_label=" + std::string(bb.label) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_raw_x=" + std::to_string(static_cast<int>(bb.x)) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_raw_y=" + std::to_string(static_cast<int>(bb.y)) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_raw_width=" + std::to_string(static_cast<int>(bb.width)) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_raw_height=" + std::to_string(static_cast<int>(bb.height)) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_confidence=" + std::to_string(bb.value) + "\n";
    if (bb.value < sharedcfg::kEdgeImpulseMinConfidence) {
      detection.rawEiOutput += "box_" + std::to_string(i) + "_rejected=low_confidence\n";
      continue;
    }
    int x1 = std::max(0, static_cast<int>(bb.x));
    int y1 = std::max(0, static_cast<int>(bb.y));
    int x2 = std::min(frame.width - 1, static_cast<int>(bb.x + bb.width) - 1);
    int y2 = std::min(frame.height - 1, static_cast<int>(bb.y + bb.height) - 1);
    padBoundingBox(x1, y1, x2, y2, frame.width, frame.height);

    redbox::RedClusterCheckResult cluster =
        redbox::evaluateRedClusterInBox(frame.rgb565.data(), frame.width, frame.height, x1, y1, x2, y2);
    for (int expansionStep = 0; expansionStep < sharedcfg::kEiClusterEdgeExpansionSteps; ++expansionStep) {
      const int currentWidth = x2 - x1 + 1;
      const int currentHeight = y2 - y1 + 1;
      if (!clusterTouchesBoxEdge(cluster, currentWidth, currentHeight)) {
        break;
      }
      const int oldX1 = x1;
      const int oldY1 = y1;
      const int oldX2 = x2;
      const int oldY2 = y2;
      expandBoundingBox(x1, y1, x2, y2, frame.width, frame.height);
      if (x1 == oldX1 && y1 == oldY1 && x2 == oldX2 && y2 == oldY2) {
        break;
      }
      cluster = redbox::evaluateRedClusterInBox(frame.rgb565.data(), frame.width, frame.height, x1, y1, x2, y2);
    }
    detection.rawEiOutput += "box_" + std::to_string(i) + "_expanded_x_min=" + std::to_string(x1) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_expanded_y_min=" + std::to_string(y1) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_expanded_x_max=" + std::to_string(x2) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_expanded_y_max=" + std::to_string(y2) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_warm_pixels=" + std::to_string(cluster.warmPixels) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_largest_warm_cluster=" + std::to_string(cluster.largestCluster) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_cluster_min_x=" + std::to_string(cluster.clusterMinX) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_cluster_min_y=" + std::to_string(cluster.clusterMinY) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_cluster_max_x=" + std::to_string(cluster.clusterMaxX) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_cluster_max_y=" + std::to_string(cluster.clusterMaxY) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_core_min_x=" + std::to_string(cluster.coreMinX) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_core_min_y=" + std::to_string(cluster.coreMinY) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_core_max_x=" + std::to_string(cluster.coreMaxX) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_core_max_y=" + std::to_string(cluster.coreMaxY) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_brightest_x=" + std::to_string(cluster.brightestX) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_brightest_y=" + std::to_string(cluster.brightestY) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_brightest_luma=" + std::to_string(cluster.brightestLuma) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_cluster_aspect_ratio=" + std::to_string(cluster.clusterAspectRatio) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_cluster_fill_fraction=" + std::to_string(cluster.clusterFillFraction) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_cluster_accepted=" + std::to_string(cluster.accepted ? 1 : 0) + "\n";
    DetectionBox box;
    box.label = bb.label;
    box.rawX = static_cast<int>(bb.x);
    box.rawY = static_cast<int>(bb.y);
    box.rawWidth = static_cast<int>(bb.width);
    box.rawHeight = static_cast<int>(bb.height);
    box.searchX1 = x1;
    box.searchY1 = y1;
    box.searchX2 = x2;
    box.searchY2 = y2;
    box.confidence = bb.value;
    box.cluster = cluster;
    if (!cluster.accepted) {
      detection.confidentBoxes.push_back(box);
      detection.rawEiOutput += "box_" + std::to_string(i) + "_rejected=cluster_not_round_enough\n";
      continue;
    }
    const redbox::SelectedRedRegion selectedRegion =
        redbox::selectRedRegionFromCluster(x1, y1, x2, y2, cluster, frame.width, frame.height);
    detection.rawEiOutput += "box_" + std::to_string(i) + "_selected_x_min=" +
                             std::to_string(selectedRegion.minX) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_selected_y_min=" +
                             std::to_string(selectedRegion.minY) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_selected_x_max=" +
                             std::to_string(selectedRegion.maxX) + "\n";
    detection.rawEiOutput += "box_" + std::to_string(i) + "_selected_y_max=" +
                             std::to_string(selectedRegion.maxY) + "\n";

    box.adjustedX1 = selectedRegion.minX;
    box.adjustedY1 = selectedRegion.minY;
    box.adjustedX2 = selectedRegion.maxX;
    box.adjustedY2 = selectedRegion.maxY;
    box.centerX = selectedRegion.centerX;
    box.centerY = selectedRegion.centerY;
    box.accepted = true;
    detection.confidentBoxes.push_back(box);

    if (!found || box.rawY < bestTopY || (box.rawY == bestTopY && box.confidence > bestScore)) {
      found = true;
      bestScore = box.confidence;
      bestTopY = box.rawY;
      detection.bestBox = box;
    }
  }

  detection.redFound = found;
  detection.confidence = bestScore;
  if (found) {
    detection.rawEiOutput += "selected_box_center_x=" + std::to_string(detection.bestBox.centerX) + "\n";
    detection.rawEiOutput += "selected_box_center_y=" + std::to_string(detection.bestBox.centerY) + "\n";
    detection.rawEiOutput += "selected_box_x_min=" + std::to_string(detection.bestBox.adjustedX1) + "\n";
    detection.rawEiOutput += "selected_box_y_min=" + std::to_string(detection.bestBox.adjustedY1) + "\n";
    detection.rawEiOutput += "selected_box_x_max=" + std::to_string(detection.bestBox.adjustedX2) + "\n";
    detection.rawEiOutput += "selected_box_y_max=" + std::to_string(detection.bestBox.adjustedY2) + "\n";
  }
  return detection;
}

void buildPatchSignature(const ImageFrame& frame, int patchX, int patchY, int patchWidth, int patchHeight,
                         uint8_t* outSignature) {
  for (int gy = 0; gy < kPatchSignatureSize; ++gy) {
    for (int gx = 0; gx < kPatchSignatureSize; ++gx) {
      const int sampleX = std::clamp(
          patchX + (gx * std::max(1, patchWidth - 1)) / std::max(1, kPatchSignatureSize - 1), 0,
          frame.width - 1);
      const int sampleY = std::clamp(
          patchY + (gy * std::max(1, patchHeight - 1)) / std::max(1, kPatchSignatureSize - 1), 0,
          frame.height - 1);
      int r = 0;
      int g = 0;
      int b = 0;
      unpackRgb565(frame.rgb565.data(), frame.width, sampleX, sampleY, r, g, b);
      outSignature[gy * kPatchSignatureSize + gx] = static_cast<uint8_t>(computeLuma(r, g, b));
    }
  }
}

float computeSceneDifference(const ImageFrame& frame, const GreenWatchState& watch) {
  uint8_t currentPatch[kPatchSignatureSize * kPatchSignatureSize]{};
  buildPatchSignature(frame, watch.patchX, watch.patchY, watch.patchWidth, watch.patchHeight, currentPatch);
  int totalDiff = 0;
  for (int i = 0; i < kPatchSignatureSize * kPatchSignatureSize; ++i) {
    totalDiff += std::abs(static_cast<int>(currentPatch[i]) - static_cast<int>(watch.patchSignature[i]));
  }
  return static_cast<float>(totalDiff) /
         static_cast<float>(kPatchSignatureSize * kPatchSignatureSize);
}

void refreshGreenWatchPatchSignature(const ImageFrame& frame, GreenWatchState& watch) {
  buildPatchSignature(frame, watch.patchX, watch.patchY, watch.patchWidth, watch.patchHeight, watch.patchSignature);
}

void startGreenWatch(const ImageFrame& frame, const DetectionBox& box, uint64_t nowMs, GreenWatchState& watch) {
  watch = GreenWatchState{};
  watch.active = true;
  watch.startedAtMs = nowMs;
  watch.lastCheckAtMs = nowMs;
  watch.sceneCheckReadyAtMs = nowMs + sharedcfg::kGreenWatchSceneChangeGraceMs;
  watch.frameWidth = frame.width;
  watch.frameHeight = frame.height;
  watch.red = box;

  int centerR = 0;
  int centerG = 0;
  int centerB = 0;
  unpackRgb565(frame.rgb565.data(), frame.width, box.centerX, box.centerY, centerR, centerG, centerB);
  watch.baselineCenterR = centerR;
  watch.baselineCenterG = centerG;
  watch.baselineCenterB = centerB;
  watch.baselineCenterLuma = computeLuma(centerR, centerG, centerB);

  const int boxWidth = std::max(1, box.adjustedX2 - box.adjustedX1 + 1);
  const int boxHeight = std::max(1, box.adjustedY2 - box.adjustedY1 + 1);
  const int coreWidth = std::max(3, (boxWidth + 1) / 2);
  const int coreHeight = std::max(3, (boxHeight + 1) / 2);
  const int coreX = std::clamp(box.centerX - coreWidth / 2, box.adjustedX1, box.adjustedX2);
  const int coreY = std::clamp(box.centerY - coreHeight / 2, box.adjustedY1, box.adjustedY2);
  const int coreMaxX = std::min(box.adjustedX2, coreX + coreWidth - 1);
  const int coreMaxY = std::min(box.adjustedY2, coreY + coreHeight - 1);
  watch.coreX = coreX;
  watch.coreY = coreY;
  watch.coreWidth = coreMaxX - coreX + 1;
  watch.coreHeight = coreMaxY - coreY + 1;

  const int patchWidth =
      std::max(boxWidth, static_cast<int>(std::ceil(boxWidth * sharedcfg::kGreenWatchPatchScale)));
  const int patchHeight =
      std::max(boxHeight, static_cast<int>(std::ceil(boxHeight * sharedcfg::kGreenWatchPatchScale)));
  watch.patchX = std::clamp(box.centerX - patchWidth / 2, 0, std::max(0, frame.width - patchWidth));
  watch.patchY = std::clamp(box.centerY - patchHeight / 2, 0, std::max(0, frame.height - patchHeight));
  watch.patchWidth = std::min(frame.width - watch.patchX, patchWidth);
  watch.patchHeight = std::min(frame.height - watch.patchY, patchHeight);
  buildPatchSignature(frame, watch.patchX, watch.patchY, watch.patchWidth, watch.patchHeight, watch.patchSignature);
}

bool findSimilarRedPixelNearCenter(const ImageFrame& frame, const GreenWatchState& watch,
                                   int originX, int originY, int& foundX, int& foundY, int& foundR,
                                   int& foundG, int& foundB, int& foundLuma) {
  const int halfBox = sharedcfg::kGreenWatchCenterSearchBoxSize / 2;
  const int minX = std::max(0, originX - halfBox);
  const int maxX = std::min(frame.width - 1, minX + sharedcfg::kGreenWatchCenterSearchBoxSize - 1);
  const int minY = std::max(0, originY - halfBox);
  const int maxY = std::min(frame.height - 1, minY + sharedcfg::kGreenWatchCenterSearchBoxSize - 1);

  bool found = false;
  int bestScore = std::numeric_limits<int>::max();
  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      int r = 0;
      int g = 0;
      int b = 0;
      unpackRgb565(frame.rgb565.data(), frame.width, x, y, r, g, b);
      if (!isRedPixel(r, g, b)) {
        continue;
      }
      const int redDelta = std::abs(r - watch.baselineCenterR);
      const int greenDelta = std::abs(g - watch.baselineCenterG);
      const int blueDelta = std::abs(b - watch.baselineCenterB);
      if (redDelta > sharedcfg::kGreenWatchSimilarRedDelta ||
          greenDelta > sharedcfg::kGreenWatchSimilarGreenDelta ||
          blueDelta > sharedcfg::kGreenWatchSimilarBlueDelta) {
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

GreenWatchUpdateResult updateGreenWatch(const ImageFrame& frame, uint64_t nowMs, GreenWatchState& watch) {
  if (!watch.active) {
    return GreenWatchUpdateResult::None;
  }
  if (nowMs - watch.startedAtMs >= sharedcfg::kGreenWatchTimeoutMs) {
    watch = GreenWatchState{};
    return GreenWatchUpdateResult::None;
  }
  if (nowMs - watch.lastCheckAtMs < sharedcfg::kGreenWatchCheckIntervalMs) {
    return GreenWatchUpdateResult::None;
  }
  watch.lastCheckAtMs = nowMs;

  const int originX = std::clamp(watch.red.centerX, 0, frame.width - 1);
  const int originY = std::clamp(watch.red.centerY, 0, frame.height - 1);
  int centerX = originX;
  int centerY = originY;
  int centerR = 0;
  int centerG = 0;
  int centerB = 0;
  int centerLuma = 0;
  const bool foundSimilarPixel =
      findSimilarRedPixelNearCenter(frame, watch, originX, originY, centerX, centerY, centerR, centerG, centerB, centerLuma);
  const bool centerDark = centerLuma <= sharedcfg::kGreenWatchCenterDarkLumaThreshold;
  const bool centerDropped =
      centerLuma <= std::max(0, watch.baselineCenterLuma - sharedcfg::kGreenWatchCenterDarkDelta);
  const float sceneDiff = computeSceneDifference(frame, watch);

  std::cout << "GREEN_WATCH file=" << frame.name
            << " found_similar=" << (foundSimilarPixel ? 1 : 0)
            << " origin=(" << originX << "," << originY << ")"
            << " center=(" << centerX << "," << centerY << ")"
            << " rgb=(" << centerR << "," << centerG << "," << centerB << ")"
            << " luma=" << centerLuma
            << " baseline_luma=" << watch.baselineCenterLuma
            << " center_dark=" << (centerDark ? 1 : 0)
            << " center_dropped=" << (centerDropped ? 1 : 0)
            << " scene_diff=" << sceneDiff
            << "\n";

  if (foundSimilarPixel) {
    if (nowMs < watch.sceneCheckReadyAtMs) {
      refreshGreenWatchPatchSignature(frame, watch);
    } else if (sceneDiff > sharedcfg::kGreenWatchSceneDiffThreshold) {
      watch = GreenWatchState{};
      return GreenWatchUpdateResult::SceneChanged;
    }
    return GreenWatchUpdateResult::None;
  }

  if (nowMs < watch.sceneCheckReadyAtMs) {
    refreshGreenWatchPatchSignature(frame, watch);
  } else if (sceneDiff > sharedcfg::kGreenWatchSceneDiffThreshold) {
    watch = GreenWatchState{};
    return GreenWatchUpdateResult::SceneChanged;
  }

  watch = GreenWatchState{};
  return GreenWatchUpdateResult::RedOffDetected;
}

void drawRect(cv::Mat& image, int x1, int y1, int x2, int y2, const cv::Scalar& color, int thickness = 1) {
  cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), color, thickness);
}

void annotateFrame(cv::Mat& image, const SequenceResult& result, const GreenWatchState&) {
  for (const auto& box : result.detection.confidentBoxes) {
    drawRect(image, box.searchX1, box.searchY1, box.searchX2, box.searchY2, cv::Scalar(0, 255, 0), 1);
    if (box.accepted) {
      drawRect(image, box.adjustedX1, box.adjustedY1, box.adjustedX2, box.adjustedY2, cv::Scalar(0, 255, 255), 1);
    }
  }
}

cv::Mat makeAnnotatedRedLightBoxCrop(const ImageFrame& frame, const DetectionBox& box) {
  const int cropWidth = box.adjustedX2 - box.adjustedX1 + 1;
  const int cropHeight = box.adjustedY2 - box.adjustedY1 + 1;
  cv::Rect roi(box.adjustedX1, box.adjustedY1, cropWidth, cropHeight);
  cv::Mat crop = frame.bgr(roi).clone();

  const int clusterX1 = std::clamp(box.searchX1 + box.cluster.clusterMinX - box.adjustedX1, 0, crop.cols - 1);
  const int clusterY1 = std::clamp(box.searchY1 + box.cluster.clusterMinY - box.adjustedY1, 0, crop.rows - 1);
  const int clusterX2 = std::clamp(box.searchX1 + box.cluster.clusterMaxX - box.adjustedX1, clusterX1, crop.cols - 1);
  const int clusterY2 = std::clamp(box.searchY1 + box.cluster.clusterMaxY - box.adjustedY1, clusterY1, crop.rows - 1);
  const int coreX1 = std::clamp(box.searchX1 + box.cluster.coreMinX - box.adjustedX1, 0, crop.cols - 1);
  const int coreY1 = std::clamp(box.searchY1 + box.cluster.coreMinY - box.adjustedY1, 0, crop.rows - 1);
  const int coreX2 = std::clamp(box.searchX1 + box.cluster.coreMaxX - box.adjustedX1, coreX1, crop.cols - 1);
  const int coreY2 = std::clamp(box.searchY1 + box.cluster.coreMaxY - box.adjustedY1, coreY1, crop.rows - 1);
  const int localCenterX = std::clamp(box.centerX - box.adjustedX1, 0, crop.cols - 1);
  const int localCenterY = std::clamp(box.centerY - box.adjustedY1, 0, crop.rows - 1);
  const int localBrightestX =
      std::clamp(box.searchX1 + box.cluster.brightestX - box.adjustedX1, 0, crop.cols - 1);
  const int localBrightestY =
      std::clamp(box.searchY1 + box.cluster.brightestY - box.adjustedY1, 0, crop.rows - 1);

  drawRect(crop, clusterX1, clusterY1, clusterX2, clusterY2, cv::Scalar(0, 128, 255), 1);
  drawRect(crop, coreX1, coreY1, coreX2, coreY2, cv::Scalar(255, 0, 255), 1);
  cv::circle(crop, cv::Point(localBrightestX, localBrightestY), 2, cv::Scalar(255, 255, 0), -1);
  cv::circle(crop, cv::Point(localCenterX, localCenterY), 2, cv::Scalar(0, 0, 255), -1);
  return crop;
}

cv::Mat makeSearchBoxCrop(const ImageFrame& frame, const DetectionBox& box) {
  const int cropWidth = box.searchX2 - box.searchX1 + 1;
  const int cropHeight = box.searchY2 - box.searchY1 + 1;
  cv::Rect roi(box.searchX1, box.searchY1, cropWidth, cropHeight);
  return frame.bgr(roi).clone();
}

std::map<fs::path, std::vector<fs::path>> listInputImageGroups(const fs::path& inputDir) {
  std::map<fs::path, std::vector<fs::path>> groups;
  for (const auto& entry : fs::recursive_directory_iterator(inputDir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    if (!isImagePath(entry.path())) {
      continue;
    }
    fs::path relativeParent = fs::relative(entry.path().parent_path(), inputDir);
    if (relativeParent.empty()) {
      relativeParent = ".";
    }
    groups[relativeParent].push_back(entry.path());
  }
  for (auto& [_, paths] : groups) {
    std::sort(paths.begin(), paths.end());
  }
  return groups;
}

std::string greenWatchResultName(GreenWatchUpdateResult result) {
  switch (result) {
    case GreenWatchUpdateResult::None:
      return "none";
    case GreenWatchUpdateResult::RedOffDetected:
      return "green_observed";
    case GreenWatchUpdateResult::SceneChanged:
      return "scene_changed";
  }
  return "unknown";
}

}  // namespace

EI_IMPULSE_ERROR ei_run_impulse_check_canceled() { return EI_IMPULSE_OK; }
void ei_serial_set_baudrate(int) {}
EI_IMPULSE_ERROR ei_sleep(int32_t) { return EI_IMPULSE_OK; }
uint64_t ei_read_timer_ms() { return 0; }
uint64_t ei_read_timer_us() { return 0; }
void ei_putchar(char c) { std::putchar(c); }
char ei_getchar(void) { return 0; }
void ei_printf(const char* format, ...) {
  va_list args;
  va_start(args, format);
  std::vfprintf(stdout, format, args);
  va_end(args);
}
void ei_printf_float(float f) { std::printf("%f", static_cast<double>(f)); }
void* ei_malloc(size_t size) { return std::malloc(size); }
void* ei_calloc(size_t nitems, size_t size) { return std::calloc(nitems, size); }
void ei_free(void* ptr) { std::free(ptr); }

int main(int argc, char** argv) {
  try {
    const fs::path root = argc > 1 ? fs::path(argv[1]) : fs::path("/Users/jamesbowler/esp32/tests/end_to_end/input");
    const fs::path outputDir = argc > 2 ? fs::path(argv[2]) : fs::path("/Users/jamesbowler/esp32/tests/end_to_end/out");
    if (!fs::exists(root)) {
      std::cerr << "Input dir not found: " << root << "\n";
      return 1;
    }
    fs::create_directories(outputDir);

    const auto inputGroups = listInputImageGroups(root);
    size_t totalInputCount = 0;
    for (const auto& [_, paths] : inputGroups) {
      totalInputCount += paths.size();
    }
    if (totalInputCount == 0) {
      std::cerr << "No input images found in " << root << "\n";
      return 1;
    }

    std::cout << "end_to_end input_dir=" << root << " cases=" << inputGroups.size()
              << " files=" << totalInputCount << "\n";
    std::cout << "annotations: green_box=padded_ei_search_box cyan_box=accepted_selected_padded_box\n";

    for (const auto& [relativeCaseDir, inputPaths] : inputGroups) {
      const fs::path caseOutputDir =
          relativeCaseDir == "." ? outputDir : (outputDir / relativeCaseDir);
      const std::string caseName =
          relativeCaseDir == "." ? "root" : relativeCaseDir.filename().string();
      fs::create_directories(caseOutputDir);
      std::cout << "case=" << relativeCaseDir << " files=" << inputPaths.size() << "\n";

      GreenWatchState watch;
      uint64_t syntheticNowMs = 0;
      for (const auto& path : inputPaths) {
        ImageFrame frame = loadFrame(path);
        uint64_t nowMs = frame.timestampMs != 0 ? frame.timestampMs : syntheticNowMs;
        syntheticNowMs = nowMs + 100;

        SequenceResult sequence;
        sequence.fileName = frame.name;
        sequence.timestampMs = nowMs;
        const bool wasWatching = watch.active;

        if (wasWatching) {
          sequence.greenWatchActive = true;
          sequence.greenWatchResult = updateGreenWatch(frame, nowMs, watch);
        }

        if (!wasWatching) {
          sequence.detection = detectRedLight(frame);
          if (sequence.detection.redFound) {
            startGreenWatch(frame, sequence.detection.bestBox, nowMs, watch);
          }
        }

        cv::Mat annotated = frame.bgr.clone();
        annotateFrame(annotated, sequence, watch);
        const fs::path outPath = caseOutputDir / path.filename();
        cv::imwrite(outPath.string(), annotated);
        const fs::path rawOutPath = caseOutputDir / (path.stem().string() + ".txt");
        std::ofstream(rawOutPath) << sequence.detection.rawEiOutput;
        fs::path redBoxOutPath;
        if (sequence.detection.redFound) {
          cv::Mat redBoxCrop = makeAnnotatedRedLightBoxCrop(frame, sequence.detection.bestBox);
          redBoxOutPath = caseOutputDir / ("red_light_box_" + caseName + ".png");
          cv::imwrite(redBoxOutPath.string(), redBoxCrop);
        }
        fs::path paddedSearchOutPath;
        if (!sequence.detection.confidentBoxes.empty()) {
          auto bestConfidentIt = std::max_element(
              sequence.detection.confidentBoxes.begin(), sequence.detection.confidentBoxes.end(),
              [](const DetectionBox& a, const DetectionBox& b) { return a.confidence < b.confidence; });
          paddedSearchOutPath = outputDir / ("red_light_box_" + caseName + ".png");
          cv::Mat paddedSearchCrop = makeSearchBoxCrop(frame, *bestConfidentIt);
          cv::imwrite(paddedSearchOutPath.string(), paddedSearchCrop);
        }

        std::cout << "file=" << sequence.fileName
                  << " timestamp_ms=" << sequence.timestampMs
                  << " red_found=" << (sequence.detection.redFound ? 1 : 0)
                  << " confidence=" << sequence.detection.confidence
                  << " inference_ms=" << sequence.detection.inferenceDurationMs
                  << " boxes=" << sequence.detection.boundingBoxCount
                  << " green_watch_active=" << (watch.active ? 1 : 0)
                  << " green_watch_result=" << greenWatchResultName(sequence.greenWatchResult)
                  << " raw_ei=" << rawOutPath;
        if (sequence.detection.redFound) {
          const auto& box = sequence.detection.bestBox;
          std::cout << " raw_box=(" << box.rawX << "," << box.rawY << "," << box.rawWidth << "," << box.rawHeight << ")"
                    << " adjusted_box=(" << box.adjustedX1 << "," << box.adjustedY1 << "," << box.adjustedX2 << "," << box.adjustedY2 << ")"
                    << " center=(" << box.centerX << "," << box.centerY << ")"
                    << " red_light_box=" << redBoxOutPath;
        }
        if (!paddedSearchOutPath.empty()) {
          std::cout << " search_box_crop=" << paddedSearchOutPath;
        }
        std::cout << " annotated=" << outPath << "\n";
      }
    }

    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "end_to_end error: " << ex.what() << "\n";
    return 1;
  }
}
