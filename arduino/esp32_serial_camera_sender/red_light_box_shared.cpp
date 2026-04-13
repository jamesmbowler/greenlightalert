#include "red_light_box_shared.h"
#include "shared_detection_config.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace redbox {
namespace {

void unpackRgb565(const uint8_t* data, int width, int x, int y, int& r, int& g, int& b) {
  const size_t index = static_cast<size_t>((y * width + x) * 2);
  const uint16_t value =
      static_cast<uint16_t>((static_cast<uint16_t>(data[index]) << 8) | data[index + 1]);
  r = ((value >> 11) & 0x1F) * 255 / 31;
  g = ((value >> 5) & 0x3F) * 255 / 63;
  b = (value & 0x1F) * 255 / 31;
}

int computeLuma(int r, int g, int b) {
  return (77 * r + 150 * g + 29 * b) >> 8;
}

bool isRedPixel(int r, int g, int b) {
  return r >= sharedcfg::kMinRedComponent &&
         (r - g) >= sharedcfg::kMinRedExcessOverGreen &&
         (r - b) >= sharedcfg::kMinRedExcessOverBlue;
}

bool isWarmBrightPixel(int r, int g, int b) {
  return r >= sharedcfg::kEiWarmBrightMinRedComponent &&
         g >= sharedcfg::kEiWarmBrightMinGreenComponent &&
         r >= g &&
         g >= b &&
         b <= sharedcfg::kEiWarmBrightMaxBlueComponent &&
         (g - b) >= sharedcfg::kEiWarmBrightMinBlueGap;
}

bool isHotCenterPixel(int r, int g, int b) {
  return r >= sharedcfg::kEiHotCenterMinRedComponent &&
         g >= sharedcfg::kEiHotCenterMinGreenComponent &&
         b >= sharedcfg::kEiHotCenterMinBlueComponent &&
         r >= g &&
         r >= b &&
         (r - g) >= sharedcfg::kEiHotCenterMinRedLead &&
         (r - b) >= sharedcfg::kEiHotCenterMinRedLead &&
         std::abs(g - b) <= sharedcfg::kEiHotCenterMaxGreenBlueGap;
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

}  // namespace

RedClusterCheckResult evaluateRedClusterInBox(const uint8_t* rgb565, int width, int height,
                                              int minX, int minY, int maxX, int maxY) {
  RedClusterCheckResult result;
  if (rgb565 == nullptr || width <= 0 || height <= 0) {
    return result;
  }

  const int x1 = std::clamp(minX, 0, width - 1);
  const int y1 = std::clamp(minY, 0, height - 1);
  const int x2 = std::clamp(maxX, x1, width - 1);
  const int y2 = std::clamp(maxY, y1, height - 1);
  const int boxWidth = x2 - x1 + 1;
  const int boxHeight = y2 - y1 + 1;
  result.samplePixels = boxWidth * boxHeight;
  if (result.samplePixels <= 0) {
    return result;
  }

  std::vector<uint8_t> candidateMask(static_cast<size_t>(result.samplePixels), 0);
  std::vector<uint8_t> redSeedMask(static_cast<size_t>(result.samplePixels), 0);
  for (int y = y1; y <= y2; ++y) {
    for (int x = x1; x <= x2; ++x) {
      int r = 0;
      int g = 0;
      int b = 0;
      unpackRgb565(rgb565, width, x, y, r, g, b);
      const bool isRed = isRedPixel(r, g, b);
      const bool isHotCenter = isHotCenterPixel(r, g, b);
      const bool isWarmBright = isWarmBrightPixel(r, g, b);
      if (!isRed && !isHotCenter) {
        continue;
      }
      const size_t index = static_cast<size_t>(y - y1) * static_cast<size_t>(boxWidth) +
                           static_cast<size_t>(x - x1);
      candidateMask[index] = 1;
      redSeedMask[index] = (isRed || isHotCenter) ? 1 : 0;
      ++result.warmPixels;
    }
  }

  if (result.warmPixels == 0) {
    return result;
  }

  std::vector<uint8_t> visited(candidateMask.size(), 0);
  std::vector<int> queue;
  queue.reserve(candidateMask.size());
  float bestClusterScore = -1.0f;
  for (int localY = 0; localY < boxHeight; ++localY) {
    for (int localX = 0; localX < boxWidth; ++localX) {
      const int startIndex = localY * boxWidth + localX;
      if (redSeedMask[static_cast<size_t>(startIndex)] == 0 ||
          visited[static_cast<size_t>(startIndex)] != 0) {
        continue;
      }

      int clusterSize = 0;
      int clusterMinX = localX;
      int clusterMinY = localY;
      int clusterMaxX = localX;
      int clusterMaxY = localY;
      int64_t clusterSumX = 0;
      int64_t clusterSumY = 0;
      int clusterCoreMinX = localX;
      int clusterCoreMinY = localY;
      int clusterCoreMaxX = localX;
      int clusterCoreMaxY = localY;
      bool coreFound = false;
      int clusterBrightestX = localX;
      int clusterBrightestY = localY;
      int clusterBrightestLuma = -1;
      queue.clear();
      queue.push_back(startIndex);
      visited[static_cast<size_t>(startIndex)] = 1;
      for (size_t queueIndex = 0; queueIndex < queue.size(); ++queueIndex) {
        const int current = queue[queueIndex];
        ++clusterSize;
        const int currentX = current % boxWidth;
        const int currentY = current / boxWidth;
        clusterSumX += currentX;
        clusterSumY += currentY;
        clusterMinX = std::min(clusterMinX, currentX);
        clusterMinY = std::min(clusterMinY, currentY);
        clusterMaxX = std::max(clusterMaxX, currentX);
        clusterMaxY = std::max(clusterMaxY, currentY);
        int r = 0;
        int g = 0;
        int b = 0;
        unpackRgb565(rgb565, width, x1 + currentX, y1 + currentY, r, g, b);
        const int luma = computeLuma(r, g, b);
        if (luma > clusterBrightestLuma) {
          clusterBrightestLuma = luma;
          clusterBrightestX = currentX;
          clusterBrightestY = currentY;
        }
        if (isWarmBrightPixel(r, g, b) || isHotCenterPixel(r, g, b)) {
          if (!coreFound) {
            clusterCoreMinX = currentX;
            clusterCoreMinY = currentY;
            clusterCoreMaxX = currentX;
            clusterCoreMaxY = currentY;
            coreFound = true;
          } else {
            clusterCoreMinX = std::min(clusterCoreMinX, currentX);
            clusterCoreMinY = std::min(clusterCoreMinY, currentY);
            clusterCoreMaxX = std::max(clusterCoreMaxX, currentX);
            clusterCoreMaxY = std::max(clusterCoreMaxY, currentY);
          }
        }
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const int nextX = currentX + dx;
            const int nextY = currentY + dy;
            if (nextX < 0 || nextX >= boxWidth || nextY < 0 || nextY >= boxHeight) {
              continue;
            }
            const int nextIndex = nextY * boxWidth + nextX;
            if (candidateMask[static_cast<size_t>(nextIndex)] == 0 ||
                visited[static_cast<size_t>(nextIndex)] != 0) {
              continue;
            }
            visited[static_cast<size_t>(nextIndex)] = 1;
            queue.push_back(nextIndex);
          }
        }
      }
      const int thisClusterWidth = std::max(1, clusterMaxX - clusterMinX + 1);
      const int thisClusterHeight = std::max(1, clusterMaxY - clusterMinY + 1);
      const int thisClusterArea = thisClusterWidth * thisClusterHeight;
      const float thisClusterAspectRatio =
          static_cast<float>(std::max(thisClusterWidth, thisClusterHeight)) /
          static_cast<float>(std::max(1, std::min(thisClusterWidth, thisClusterHeight)));
      const float thisClusterFillFraction =
          static_cast<float>(clusterSize) / static_cast<float>(std::max(1, thisClusterArea));
      const bool thisClusterAccepted =
          clusterSize >= sharedcfg::kEiCandidateMinRedClusterPixels &&
          thisClusterAspectRatio <= sharedcfg::kEiCandidateMaxClusterAspectRatio &&
          thisClusterFillFraction >= sharedcfg::kEiCandidateMinClusterFillFraction;
      const float thisClusterScore =
          thisClusterAccepted ? (thisClusterFillFraction / std::max(1.0f, thisClusterAspectRatio) +
                                 static_cast<float>(clusterSize) * 0.01f)
                              : -1.0f;
      if (thisClusterScore > bestClusterScore) {
        bestClusterScore = thisClusterScore;
        result.largestCluster = clusterSize;
        result.clusterMinX = clusterMinX;
        result.clusterMinY = clusterMinY;
        result.clusterMaxX = clusterMaxX;
        result.clusterMaxY = clusterMaxY;
        result.clusterCenterX =
            static_cast<int>(std::lround(static_cast<double>(clusterSumX) / clusterSize));
        result.clusterCenterY =
            static_cast<int>(std::lround(static_cast<double>(clusterSumY) / clusterSize));
        result.coreMinX = coreFound ? clusterCoreMinX : clusterMinX;
        result.coreMinY = coreFound ? clusterCoreMinY : clusterMinY;
        result.coreMaxX = coreFound ? clusterCoreMaxX : clusterMaxX;
        result.coreMaxY = coreFound ? clusterCoreMaxY : clusterMaxY;
        result.brightestX = clusterBrightestX;
        result.brightestY = clusterBrightestY;
        result.brightestLuma = clusterBrightestLuma;
        result.clusterAspectRatio = thisClusterAspectRatio;
        result.clusterFillFraction = thisClusterFillFraction;
        result.accepted = thisClusterAccepted;
      }
    }
  }

  return result;
}

SelectedRedRegion selectRedRegionFromCluster(int searchMinX, int searchMinY, int searchMaxX,
                                             int searchMaxY,
                                             const RedClusterCheckResult& cluster,
                                             int frameWidth, int frameHeight) {
  SelectedRedRegion region;
  region.minX = std::clamp(searchMinX + cluster.clusterMinX, 0, std::max(0, frameWidth - 1));
  region.minY = std::clamp(searchMinY + cluster.clusterMinY, 0, std::max(0, frameHeight - 1));
  region.maxX = std::clamp(searchMinX + cluster.clusterMaxX, region.minX,
                           std::max(0, frameWidth - 1));
  region.maxY = std::clamp(searchMinY + cluster.clusterMaxY, region.minY,
                           std::max(0, frameHeight - 1));
  padBoundingBox(region.minX, region.minY, region.maxX, region.maxY, frameWidth, frameHeight);

  const int coreMidX = searchMinX + (cluster.coreMinX + cluster.coreMaxX) / 2;
  const int coreHeight = std::max(1, cluster.coreMaxY - cluster.coreMinY + 1);
  const int lowerBiasedCoreY = searchMinY + cluster.coreMinY + ((coreHeight - 1) * 2) / 3;
  region.centerX = std::clamp(coreMidX, region.minX, region.maxX);
  region.centerY = std::clamp(lowerBiasedCoreY, region.minY, region.maxY);
  return region;
}

}  // namespace redbox
