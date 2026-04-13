#pragma once

#include <cstdint>

namespace redbox {

struct RedClusterCheckResult {
  bool accepted = false;
  int largestCluster = 0;
  int warmPixels = 0;
  int samplePixels = 0;
  int clusterMinX = 0;
  int clusterMinY = 0;
  int clusterMaxX = 0;
  int clusterMaxY = 0;
  int clusterCenterX = 0;
  int clusterCenterY = 0;
  int coreMinX = 0;
  int coreMinY = 0;
  int coreMaxX = 0;
  int coreMaxY = 0;
  int brightestX = 0;
  int brightestY = 0;
  int brightestLuma = -1;
  float clusterAspectRatio = 0.0f;
  float clusterFillFraction = 0.0f;
};

struct SelectedRedRegion {
  int minX = 0;
  int minY = 0;
  int maxX = 0;
  int maxY = 0;
  int centerX = 0;
  int centerY = 0;
};

RedClusterCheckResult evaluateRedClusterInBox(const uint8_t* rgb565, int width, int height,
                                              int minX, int minY, int maxX, int maxY);

SelectedRedRegion selectRedRegionFromCluster(int searchMinX, int searchMinY, int searchMaxX,
                                             int searchMaxY,
                                             const RedClusterCheckResult& cluster,
                                             int frameWidth, int frameHeight);

}  // namespace redbox
