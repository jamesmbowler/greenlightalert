#pragma once

#include <cstdint>

namespace sharedcfg {

constexpr float kEdgeImpulseMinConfidence = 0.90f;

constexpr int kMinRedComponent = 210;
constexpr int kMinRedExcessOverGreen = 28;
constexpr int kMinRedExcessOverBlue = 28;
constexpr int kMinRedPixelsForDetection = 10;

constexpr uint32_t kGreenWatchCheckIntervalMs = 50;
constexpr uint32_t kGreenWatchTimeoutMs = 40000;
constexpr uint32_t kGreenWatchSceneChangeGraceMs = 1200;
constexpr int kGreenWatchDarkLumaThreshold = 90;
constexpr int kGreenWatchCenterDarkLumaThreshold = 90;
constexpr int kGreenWatchCenterDarkDelta = 35;
constexpr int kGreenWatchCenterSearchBoxSize = 10;
constexpr int kGreenWatchSimilarRedDelta = 55;
constexpr int kGreenWatchSimilarGreenDelta = 45;
constexpr int kGreenWatchSimilarBlueDelta = 45;
constexpr float kGreenWatchSceneDiffThreshold = 60.0f;
constexpr float kGreenWatchPatchScale = 1.25f;

constexpr int kEiCandidateMinRedClusterPixels = 6;
constexpr float kEiCandidateMaxClusterAspectRatio = 1.8f;
constexpr float kEiCandidateMinClusterFillFraction = 0.35f;
constexpr float kEiBoundingBoxPaddingFraction = 0.25f;
constexpr int kEiClusterEdgeExpansionSteps = 2;
constexpr float kEiClusterEdgeExpansionFraction = 0.20f;
constexpr int kEiWarmBrightMinRedComponent = 235;
constexpr int kEiWarmBrightMinGreenComponent = 180;
constexpr int kEiWarmBrightMaxBlueComponent = 130;
constexpr int kEiWarmBrightMinBlueGap = 55;
constexpr int kEiHotCenterMinRedComponent = 240;
constexpr int kEiHotCenterMinGreenComponent = 220;
constexpr int kEiHotCenterMinBlueComponent = 220;
constexpr int kEiHotCenterMinRedLead = 8;
constexpr int kEiHotCenterMaxGreenBlueGap = 20;

}  // namespace sharedcfg
