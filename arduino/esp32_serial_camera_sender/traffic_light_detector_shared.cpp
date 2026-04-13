#include "traffic_light_detector_shared.h"

#include <algorithm>
#include <vector>

namespace traffic {
namespace {

constexpr int kScanStride = 2;
constexpr int kGridCols = 8;
constexpr int kGridRows = 6;
constexpr int kMinRedComponent = 110;
constexpr float kMinRedDominanceOverGreen = 1.25f;
constexpr float kMinRedDominanceOverBlue = 1.20f;
constexpr int kMinRedPixelsForDetection = 6;
constexpr float kYellowYOffsetMultiplier = 2.5f;
constexpr float kGreenYOffsetMultiplier = 5.0f;
constexpr int kMinProbeRadius = 8;
constexpr int kSearchMinY = 0;
constexpr float kSearchMaxYFraction = 0.98f;
constexpr float kMaxLampCenterYFraction = 0.60f;
constexpr int kMinAllowedMaxLampCenterY = 126;
constexpr int kMinLampRadius = 2;
constexpr int kMaxLampRadius = 28;
constexpr float kMaxLampAspectRatio = 2.20f;
constexpr float kMinLampFillRatio = 0.22f;
constexpr int kDarkComponentMax = 80;
constexpr int kSoftDarkComponentMax = 136;
constexpr float kMinDarkFractionAtExpectedLamps = 0.18f;
constexpr float kMinHousingDarkFraction = 0.28f;
constexpr float kCenterSearchBandFraction = 0.18f;
constexpr float kSkyBlueMin = 90.0f;
constexpr int kSkyGrayMaxDelta = 22;
constexpr int kSkyScanStrideY = 4;
constexpr int kSkyTransitionRows = 3;
constexpr int kCenterOverrideMinPixels = 5;
constexpr int kCenterOverrideMaxBoxSize = 12;
constexpr float kCenterOverrideMaxAspectRatio = 1.45f;
constexpr float kCenterOverrideMinFillRatio = 0.45f;
constexpr float kCenterOverrideMinDarkRingFraction = 0.0f;
constexpr float kCenterOverrideMinYFraction = 0.45f;
constexpr float kCompactBypassMinYFraction = 0.60f;

void unpack_rgb565(const uint8_t* data, int index, int& r, int& g, int& b) {
  const uint16_t pixel = (static_cast<uint16_t>(data[index]) << 8) | data[index + 1];
  r = ((pixel >> 11) & 0x1F) * 255 / 31;
  g = ((pixel >> 5) & 0x3F) * 255 / 63;
  b = (pixel & 0x1F) * 255 / 31;
}

bool is_red_pixel(int r, int g, int b) {
  return r >= kMinRedComponent &&
         r >= static_cast<int>(g * kMinRedDominanceOverGreen) &&
         r >= static_cast<int>(b * kMinRedDominanceOverBlue);
}

bool is_dark_pixel(int r, int g, int b) {
  return r <= kDarkComponentMax && g <= kDarkComponentMax && b <= kDarkComponentMax;
}

bool is_soft_dark_pixel(int r, int g, int b) {
  return r <= kSoftDarkComponentMax && g <= kSoftDarkComponentMax &&
         b <= kSoftDarkComponentMax;
}

bool is_sky_or_gray_pixel(int r, int g, int b) {
  const int max_component = std::max(r, std::max(g, b));
  const int min_component = std::min(r, std::min(g, b));
  const bool gray = (max_component - min_component) <= kSkyGrayMaxDelta && max_component >= 70;
  const bool blue_sky = b >= kSkyBlueMin && b >= r && b >= g;
  return gray || blue_sky;
}

int detect_object_start_y(const Rgb565Frame& frame) {
  const int center_x = frame.width / 2;
  const int band_half_width =
      std::max(8, static_cast<int>(frame.width * kCenterSearchBandFraction * 0.5f));
  const int min_x = std::max(0, center_x - band_half_width);
  const int max_x = std::min(frame.width - 1, center_x + band_half_width);
  int consecutive_object_rows = 0;

  for (int y = 0; y < frame.height; y += kSkyScanStrideY) {
    int sky_like_count = 0;
    int sample_count = 0;
    for (int x = min_x; x <= max_x; x += kScanStride) {
      const int index = (y * frame.width + x) * 2;
      int r = 0;
      int g = 0;
      int b = 0;
      unpack_rgb565(frame.data, index, r, g, b);
      if (is_sky_or_gray_pixel(r, g, b)) {
        ++sky_like_count;
      }
      ++sample_count;
    }

    const float sky_like_fraction = sample_count == 0
                                        ? 1.0f
                                        : static_cast<float>(sky_like_count) /
                                              static_cast<float>(sample_count);
    if (sky_like_fraction < 0.60f) {
      ++consecutive_object_rows;
      if (consecutive_object_rows >= kSkyTransitionRows) {
        return std::max(0, y - (kSkyTransitionRows - 1) * kSkyScanStrideY);
      }
    } else {
      consecutive_object_rows = 0;
    }
  }

  return frame.height / 5;
}

float dark_fraction_in_region(const Rgb565Frame& frame, int center_x, int center_y, int radius) {
  const int min_x = std::max(0, center_x - radius);
  const int max_x = std::min(frame.width - 1, center_x + radius);
  const int min_y = std::max(0, center_y - radius);
  const int max_y = std::min(frame.height - 1, center_y + radius);
  const int radius_squared = radius * radius;

  int dark_count = 0;
  int total_count = 0;

  for (int y = min_y; y <= max_y; y += kScanStride) {
    for (int x = min_x; x <= max_x; x += kScanStride) {
      const int dx = x - center_x;
      const int dy = y - center_y;
      if (dx * dx + dy * dy > radius_squared) {
        continue;
      }

      const int index = (y * frame.width + x) * 2;
      int r = 0;
      int g = 0;
      int b = 0;
      unpack_rgb565(frame.data, index, r, g, b);
      if (is_dark_pixel(r, g, b)) {
        ++dark_count;
      }
      ++total_count;
    }
  }

  if (total_count == 0) {
    return 0.0f;
  }
  return static_cast<float>(dark_count) / static_cast<float>(total_count);
}

float soft_dark_fraction_in_region(const Rgb565Frame& frame, int center_x, int center_y,
                                   int radius) {
  const int min_x = std::max(0, center_x - radius);
  const int max_x = std::min(frame.width - 1, center_x + radius);
  const int min_y = std::max(0, center_y - radius);
  const int max_y = std::min(frame.height - 1, center_y + radius);
  const int radius_squared = radius * radius;

  int dark_count = 0;
  int total_count = 0;

  for (int y = min_y; y <= max_y; y += kScanStride) {
    for (int x = min_x; x <= max_x; x += kScanStride) {
      const int dx = x - center_x;
      const int dy = y - center_y;
      if (dx * dx + dy * dy > radius_squared) {
        continue;
      }

      const int index = (y * frame.width + x) * 2;
      int r = 0;
      int g = 0;
      int b = 0;
      unpack_rgb565(frame.data, index, r, g, b);
      if (is_soft_dark_pixel(r, g, b)) {
        ++dark_count;
      }
      ++total_count;
    }
  }

  if (total_count == 0) {
    return 0.0f;
  }
  return static_cast<float>(dark_count) / static_cast<float>(total_count);
}

float dark_fraction_in_vertical_housing(const Rgb565Frame& frame, const LampDetection& red) {
  const int half_width = std::max(3, red.radius);
  const int min_x = std::max(0, red.center_x - half_width);
  const int max_x = std::min(frame.width - 1, red.center_x + half_width);
  const int min_y = std::min(frame.height - 1, red.center_y + red.radius);
  const int max_y = std::min(
      frame.height - 1,
      red.center_y + static_cast<int>(std::max(6, red.radius) * kGreenYOffsetMultiplier));

  if (max_y <= min_y || max_x <= min_x) {
    return 0.0f;
  }

  int dark_count = 0;
  int total_count = 0;
  for (int y = min_y; y <= max_y; y += kScanStride) {
    for (int x = min_x; x <= max_x; x += kScanStride) {
      const int index = (y * frame.width + x) * 2;
      int r = 0;
      int g = 0;
      int b = 0;
      unpack_rgb565(frame.data, index, r, g, b);
      if (is_dark_pixel(r, g, b)) {
        ++dark_count;
      }
      ++total_count;
    }
  }

  if (total_count == 0) {
    return 0.0f;
  }
  return static_cast<float>(dark_count) / static_cast<float>(total_count);
}

bool validate_traffic_light_stack(const Rgb565Frame& frame, const LampDetection& red,
                                  DetectionResult& debug) {
  const int probe_radius = std::max(kMinProbeRadius, static_cast<int>(red.radius * 1.2f));
  const int yellow_center_y = std::min(
      frame.height - 1,
      red.center_y + static_cast<int>(std::max(6, red.radius) * kYellowYOffsetMultiplier));
  const int green_center_y = std::min(
      frame.height - 1,
      red.center_y + static_cast<int>(std::max(6, red.radius) * kGreenYOffsetMultiplier));

  const float yellow_dark_fraction =
      dark_fraction_in_region(frame, red.center_x, yellow_center_y, probe_radius);
  const float green_dark_fraction =
      dark_fraction_in_region(frame, red.center_x, green_center_y, probe_radius);
  const float housing_dark_fraction = dark_fraction_in_vertical_housing(frame, red);
  debug.yellow_dark_fraction = yellow_dark_fraction;
  debug.green_dark_fraction = green_dark_fraction;
  debug.housing_dark_fraction = housing_dark_fraction;

  return yellow_dark_fraction >= kMinDarkFractionAtExpectedLamps &&
         green_dark_fraction >= kMinDarkFractionAtExpectedLamps &&
         housing_dark_fraction >= kMinHousingDarkFraction;
}

struct CandidateComponent {
  LampDetection lamp{};
  int box_width = 0;
  int box_height = 0;
  float aspect_ratio = 0.0f;
  float fill_ratio = 0.0f;
  float dark_ring_fraction = 0.0f;
  int avg_r = 0;
  int avg_g = 0;
  int avg_b = 0;
};

void record_debug_candidate(DetectionResult& debug, const CandidateComponent& candidate) {
  for (int i = 0; i < debug.debug_candidate_count; ++i) {
    const auto& existing = debug.debug_candidates[i].lamp;
    if (std::abs(existing.center_x - candidate.lamp.center_x) <= 2 &&
        std::abs(existing.center_y - candidate.lamp.center_y) <= 2) {
      return;
    }
  }
  if (debug.debug_candidate_count >= kMaxDebugCandidates) {
    return;
  }
  auto& slot = debug.debug_candidates[debug.debug_candidate_count++];
  slot.lamp = candidate.lamp;
  slot.avg_r = candidate.avg_r;
  slot.avg_g = candidate.avg_g;
  slot.avg_b = candidate.avg_b;
}

void collect_all_debug_red_components(const Rgb565Frame& frame, DetectionResult& debug) {
  const int search_min_y = kSearchMinY;
  const int search_max_y =
      std::min(frame.height - 1, static_cast<int>(frame.height * kSearchMaxYFraction));
  if (search_max_y < search_min_y) {
    return;
  }

  const int sample_cols = ((frame.width - 1) / kScanStride) + 1;
  const int sample_rows = ((search_max_y - search_min_y) / kScanStride) + 1;
  std::vector<uint8_t> red_mask(static_cast<size_t>(sample_cols * sample_rows), 0);
  std::vector<uint8_t> visited(static_cast<size_t>(sample_cols * sample_rows), 0);

  for (int row = 0; row < sample_rows; ++row) {
    const int y = search_min_y + row * kScanStride;
    for (int col = 0; col < sample_cols; ++col) {
      const int x = col * kScanStride;
      const int index = (y * frame.width + x) * 2;
      int r = 0;
      int g = 0;
      int b = 0;
      unpack_rgb565(frame.data, index, r, g, b);
      if (is_red_pixel(r, g, b)) {
        red_mask[static_cast<size_t>(row * sample_cols + col)] = 1;
      }
    }
  }

  std::vector<int> stack;
  stack.reserve(static_cast<size_t>(sample_cols * sample_rows));
  for (int row = 0; row < sample_rows; ++row) {
    for (int col = 0; col < sample_cols; ++col) {
      const int start_index = row * sample_cols + col;
      if (!red_mask[static_cast<size_t>(start_index)] ||
          visited[static_cast<size_t>(start_index)]) {
        continue;
      }

      int component_count = 0;
      int component_sum_x = 0;
      int component_sum_y = 0;
      int component_sum_r = 0;
      int component_sum_g = 0;
      int component_sum_b = 0;
      int component_min_x = frame.width;
      int component_min_y = frame.height;
      int component_max_x = 0;
      int component_max_y = 0;

      stack.clear();
      stack.push_back(start_index);
      visited[static_cast<size_t>(start_index)] = 1;

      while (!stack.empty()) {
        const int current = stack.back();
        stack.pop_back();

        const int current_row = current / sample_cols;
        const int current_col = current % sample_cols;
        const int x = current_col * kScanStride;
        const int y = search_min_y + current_row * kScanStride;
        int r = 0;
        int g = 0;
        int b = 0;
        unpack_rgb565(frame.data, (y * frame.width + x) * 2, r, g, b);

        ++component_count;
        component_sum_x += x;
        component_sum_y += y;
        component_sum_r += r;
        component_sum_g += g;
        component_sum_b += b;
        component_min_x = std::min(component_min_x, x);
        component_min_y = std::min(component_min_y, y);
        component_max_x = std::max(component_max_x, x);
        component_max_y = std::max(component_max_y, y);

        const int neighbors[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        for (const auto& neighbor : neighbors) {
          const int next_row = current_row + neighbor[0];
          const int next_col = current_col + neighbor[1];
          if (next_row < 0 || next_row >= sample_rows || next_col < 0 ||
              next_col >= sample_cols) {
            continue;
          }
          const int next_index = next_row * sample_cols + next_col;
          if (!red_mask[static_cast<size_t>(next_index)] ||
              visited[static_cast<size_t>(next_index)]) {
            continue;
          }
          visited[static_cast<size_t>(next_index)] = 1;
          stack.push_back(next_index);
        }
      }

      CandidateComponent candidate;
      candidate.lamp.found = true;
      candidate.lamp.min_x = component_min_x;
      candidate.lamp.min_y = component_min_y;
      candidate.lamp.max_x = component_max_x;
      candidate.lamp.max_y = component_max_y;
      candidate.lamp.center_x = component_sum_x / std::max(1, component_count);
      candidate.lamp.center_y = component_sum_y / std::max(1, component_count);
      candidate.lamp.radius = std::max(component_max_x - component_min_x + kScanStride,
                                       component_max_y - component_min_y + kScanStride) / 2;
      candidate.lamp.pixel_count = component_count;
      candidate.box_width = component_max_x - component_min_x + kScanStride;
      candidate.box_height = component_max_y - component_min_y + kScanStride;
      candidate.aspect_ratio =
          static_cast<float>(std::max(candidate.box_width, candidate.box_height)) /
          static_cast<float>(std::max(1, std::min(candidate.box_width, candidate.box_height)));
      candidate.fill_ratio =
          static_cast<float>(component_count * kScanStride * kScanStride) /
          static_cast<float>(std::max(1, candidate.box_width * candidate.box_height));
      candidate.avg_r = component_sum_r / std::max(1, component_count);
      candidate.avg_g = component_sum_g / std::max(1, component_count);
      candidate.avg_b = component_sum_b / std::max(1, component_count);
      record_debug_candidate(debug, candidate);
    }
  }
}

CandidateComponent find_center_override_candidate(const Rgb565Frame& frame, DetectionResult& debug) {
  const int center_x = frame.width / 2;
  const int band_half_width =
      std::max(8, static_cast<int>(frame.width * kCenterSearchBandFraction));
  const int search_min_x = std::max(0, center_x - band_half_width);
  const int search_max_x = std::min(frame.width - 1, center_x + band_half_width);
  const int search_min_y = detect_object_start_y(frame);
  const int search_max_y =
      std::min(frame.height - 1, static_cast<int>(frame.height * kSearchMaxYFraction));

  if (search_max_y < search_min_y) {
    return {};
  }

  const int sample_cols = ((search_max_x - search_min_x) / kScanStride) + 1;
  const int sample_rows = ((search_max_y - search_min_y) / kScanStride) + 1;
  std::vector<uint8_t> red_mask(static_cast<size_t>(sample_cols * sample_rows), 0);
  std::vector<uint8_t> visited(static_cast<size_t>(sample_cols * sample_rows), 0);

  for (int row = 0; row < sample_rows; ++row) {
    const int y = search_min_y + row * kScanStride;
    for (int col = 0; col < sample_cols; ++col) {
      const int x = search_min_x + col * kScanStride;
      const int index = (y * frame.width + x) * 2;
      int r = 0;
      int g = 0;
      int b = 0;
      unpack_rgb565(frame.data, index, r, g, b);
      if (is_red_pixel(r, g, b)) {
        red_mask[static_cast<size_t>(row * sample_cols + col)] = 1;
      }
    }
  }

  CandidateComponent best;
  std::vector<int> stack;
  stack.reserve(static_cast<size_t>(sample_cols * sample_rows));
  bool found = false;

  for (int row = 0; row < sample_rows; ++row) {
    for (int col = 0; col < sample_cols; ++col) {
      const int start_index = row * sample_cols + col;
      if (!red_mask[static_cast<size_t>(start_index)] ||
          visited[static_cast<size_t>(start_index)]) {
        continue;
      }

      int component_count = 0;
      int component_sum_x = 0;
      int component_sum_y = 0;
      int component_sum_r = 0;
      int component_sum_g = 0;
      int component_sum_b = 0;
      int component_min_x = frame.width;
      int component_min_y = frame.height;
      int component_max_x = 0;
      int component_max_y = 0;

      stack.clear();
      stack.push_back(start_index);
      visited[static_cast<size_t>(start_index)] = 1;

      while (!stack.empty()) {
        const int current = stack.back();
        stack.pop_back();

        const int current_row = current / sample_cols;
        const int current_col = current % sample_cols;
        const int x = search_min_x + current_col * kScanStride;
        const int y = search_min_y + current_row * kScanStride;
        int r = 0;
        int g = 0;
        int b = 0;
        unpack_rgb565(frame.data, (y * frame.width + x) * 2, r, g, b);

        ++component_count;
        component_sum_x += x;
        component_sum_y += y;
        component_sum_r += r;
        component_sum_g += g;
        component_sum_b += b;
        component_min_x = std::min(component_min_x, x);
        component_min_y = std::min(component_min_y, y);
        component_max_x = std::max(component_max_x, x);
        component_max_y = std::max(component_max_y, y);

        const int neighbors[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        for (const auto& neighbor : neighbors) {
          const int next_row = current_row + neighbor[0];
          const int next_col = current_col + neighbor[1];
          if (next_row < 0 || next_row >= sample_rows || next_col < 0 ||
              next_col >= sample_cols) {
            continue;
          }
          const int next_index = next_row * sample_cols + next_col;
          if (!red_mask[static_cast<size_t>(next_index)] ||
              visited[static_cast<size_t>(next_index)]) {
            continue;
          }
          visited[static_cast<size_t>(next_index)] = 1;
          stack.push_back(next_index);
        }
      }

      const int box_width = component_max_x - component_min_x + kScanStride;
      const int box_height = component_max_y - component_min_y + kScanStride;
      const int radius = std::max(box_width, box_height) / 2;
      const float aspect_ratio = static_cast<float>(std::max(box_width, box_height)) /
                                 static_cast<float>(std::max(1, std::min(box_width, box_height)));
      const float fill_ratio =
          static_cast<float>(component_count * kScanStride * kScanStride) /
          static_cast<float>(std::max(1, box_width * box_height));
      const int component_center_x = component_sum_x / std::max(1, component_count);
      const int component_center_y = component_sum_y / std::max(1, component_count);
      const float dark_ring_fraction = soft_dark_fraction_in_region(
          frame, component_center_x, component_center_y, std::max(kMinProbeRadius, radius * 2));

      if (component_count < kCenterOverrideMinPixels ||
          box_width > kCenterOverrideMaxBoxSize ||
          box_height > kCenterOverrideMaxBoxSize ||
          radius < kMinLampRadius || radius > kMaxLampRadius ||
          aspect_ratio > kCenterOverrideMaxAspectRatio ||
          fill_ratio < kCenterOverrideMinFillRatio ||
          component_center_y < static_cast<int>(frame.height * kCenterOverrideMinYFraction) ||
          dark_ring_fraction < kCenterOverrideMinDarkRingFraction) {
        continue;
      }

      CandidateComponent candidate;
      candidate.lamp.found = true;
      candidate.lamp.min_x = component_min_x;
      candidate.lamp.min_y = component_min_y;
      candidate.lamp.max_x = component_max_x;
      candidate.lamp.max_y = component_max_y;
      candidate.lamp.center_x = component_center_x;
      candidate.lamp.center_y = component_center_y;
      candidate.lamp.radius = radius;
      candidate.lamp.pixel_count = component_count;
      candidate.box_width = box_width;
      candidate.box_height = box_height;
      candidate.aspect_ratio = aspect_ratio;
      candidate.fill_ratio = fill_ratio;
      candidate.dark_ring_fraction = dark_ring_fraction;
      candidate.avg_r = component_sum_r / std::max(1, component_count);
      candidate.avg_g = component_sum_g / std::max(1, component_count);
      candidate.avg_b = component_sum_b / std::max(1, component_count);
      record_debug_candidate(debug, candidate);

      if (!found || candidate.lamp.center_y < best.lamp.center_y ||
          (candidate.lamp.center_y == best.lamp.center_y &&
           std::abs(candidate.lamp.center_x - center_x) <
               std::abs(best.lamp.center_x - center_x))) {
        best = candidate;
        found = true;
      }
    }
  }

  return best;
}

LampDetection detect_red_lamp_with_grid(const Rgb565Frame& frame, DetectionResult& debug) {
  LampDetection result;
  const int cell_width = frame.width / kGridCols;
  const int cell_height = frame.height / kGridRows;
  const int search_max_y = static_cast<int>(frame.height * kSearchMaxYFraction);

  int grid_counts[kGridRows][kGridCols] = {};
  int best_cell_x = -1;
  int best_cell_y = -1;
  int best_cell_count = 0;

  for (int y = kSearchMinY; y < search_max_y; y += kScanStride) {
    for (int x = 0; x < frame.width; x += kScanStride) {
      const int index = (y * frame.width + x) * 2;
      int r = 0;
      int g = 0;
      int b = 0;
      unpack_rgb565(frame.data, index, r, g, b);
      if (!is_red_pixel(r, g, b)) {
        continue;
      }

      const int cell_x = std::min(kGridCols - 1, x / std::max(1, cell_width));
      const int cell_y = std::min(kGridRows - 1, y / std::max(1, cell_height));
      const int count = ++grid_counts[cell_y][cell_x];
      if (count > best_cell_count) {
        best_cell_count = count;
        best_cell_x = cell_x;
        best_cell_y = cell_y;
      }
    }
  }

  if (best_cell_count < kMinRedPixelsForDetection || best_cell_x < 0 || best_cell_y < 0) {
    debug.best_cell_x = best_cell_x;
    debug.best_cell_y = best_cell_y;
    debug.best_cell_count = best_cell_count;
    debug.reject_reason = "no_red_cluster";
    return result;
  }

  debug.best_cell_x = best_cell_x;
  debug.best_cell_y = best_cell_y;
  debug.best_cell_count = best_cell_count;

  const int region_min_x = std::max(0, best_cell_x * cell_width - cell_width / 2);
  const int region_max_x =
      std::min(frame.width - 1, (best_cell_x + 1) * cell_width + cell_width / 2);
  const int region_min_y = std::max(0, best_cell_y * cell_height - cell_height / 2);
  const int region_max_y =
      std::min(frame.height - 1, (best_cell_y + 1) * cell_height + cell_height / 2);

  const int sample_cols = ((region_max_x - region_min_x) / kScanStride) + 1;
  const int sample_rows = ((region_max_y - region_min_y) / kScanStride) + 1;
  std::vector<uint8_t> red_mask(static_cast<size_t>(sample_cols * sample_rows), 0);
  std::vector<uint8_t> visited(static_cast<size_t>(sample_cols * sample_rows), 0);

  for (int row = 0; row < sample_rows; ++row) {
    const int y = region_min_y + row * kScanStride;
    for (int col = 0; col < sample_cols; ++col) {
      const int x = region_min_x + col * kScanStride;
      const int index = (y * frame.width + x) * 2;
      int r = 0;
      int g = 0;
      int b = 0;
      unpack_rgb565(frame.data, index, r, g, b);
      if (is_red_pixel(r, g, b)) {
        red_mask[static_cast<size_t>(row * sample_cols + col)] = 1;
      }
    }
  }

  int best_component_count = 0;
  int sum_x = 0;
  int sum_y = 0;
  int min_x = frame.width;
  int min_y = frame.height;
  int max_x = 0;
  int max_y = 0;
  std::vector<int> stack;
  stack.reserve(static_cast<size_t>(sample_cols * sample_rows));

  for (int row = 0; row < sample_rows; ++row) {
    for (int col = 0; col < sample_cols; ++col) {
      const int start_index = row * sample_cols + col;
      if (!red_mask[static_cast<size_t>(start_index)] ||
          visited[static_cast<size_t>(start_index)]) {
        continue;
      }

      int component_count = 0;
      int component_sum_x = 0;
      int component_sum_y = 0;
      int component_sum_r = 0;
      int component_sum_g = 0;
      int component_sum_b = 0;
      int component_min_x = frame.width;
      int component_min_y = frame.height;
      int component_max_x = 0;
      int component_max_y = 0;

      stack.clear();
      stack.push_back(start_index);
      visited[static_cast<size_t>(start_index)] = 1;

      while (!stack.empty()) {
        const int current = stack.back();
        stack.pop_back();

        const int current_row = current / sample_cols;
        const int current_col = current % sample_cols;
        const int x = region_min_x + current_col * kScanStride;
        const int y = region_min_y + current_row * kScanStride;
        int r = 0;
        int g = 0;
        int b = 0;
        unpack_rgb565(frame.data, (y * frame.width + x) * 2, r, g, b);

        ++component_count;
        component_sum_x += x;
        component_sum_y += y;
        component_sum_r += r;
        component_sum_g += g;
        component_sum_b += b;
        component_min_x = std::min(component_min_x, x);
        component_min_y = std::min(component_min_y, y);
        component_max_x = std::max(component_max_x, x);
        component_max_y = std::max(component_max_y, y);

        const int neighbors[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        for (const auto& neighbor : neighbors) {
          const int next_row = current_row + neighbor[0];
          const int next_col = current_col + neighbor[1];
          if (next_row < 0 || next_row >= sample_rows || next_col < 0 ||
              next_col >= sample_cols) {
            continue;
          }
          const int next_index = next_row * sample_cols + next_col;
          if (!red_mask[static_cast<size_t>(next_index)] ||
              visited[static_cast<size_t>(next_index)]) {
            continue;
          }
          visited[static_cast<size_t>(next_index)] = 1;
          stack.push_back(next_index);
        }
      }

      CandidateComponent candidate;
      candidate.lamp.found = true;
      candidate.lamp.min_x = component_min_x;
      candidate.lamp.min_y = component_min_y;
      candidate.lamp.max_x = component_max_x;
      candidate.lamp.max_y = component_max_y;
      candidate.lamp.center_x = component_sum_x / std::max(1, component_count);
      candidate.lamp.center_y = component_sum_y / std::max(1, component_count);
      candidate.lamp.radius = std::max(component_max_x - component_min_x + kScanStride,
                                       component_max_y - component_min_y + kScanStride) / 2;
      candidate.lamp.pixel_count = component_count;
      candidate.box_width = component_max_x - component_min_x + kScanStride;
      candidate.box_height = component_max_y - component_min_y + kScanStride;
      candidate.aspect_ratio =
          static_cast<float>(std::max(candidate.box_width, candidate.box_height)) /
          static_cast<float>(std::max(1, std::min(candidate.box_width, candidate.box_height)));
      candidate.fill_ratio =
          static_cast<float>(component_count * kScanStride * kScanStride) /
          static_cast<float>(std::max(1, candidate.box_width * candidate.box_height));
      candidate.avg_r = component_sum_r / std::max(1, component_count);
      candidate.avg_g = component_sum_g / std::max(1, component_count);
      candidate.avg_b = component_sum_b / std::max(1, component_count);
      record_debug_candidate(debug, candidate);

      if (component_count > best_component_count) {
        best_component_count = component_count;
        sum_x = component_sum_x;
        sum_y = component_sum_y;
        min_x = component_min_x;
        min_y = component_min_y;
        max_x = component_max_x;
        max_y = component_max_y;
      }
    }
  }

  if (best_component_count < kMinRedPixelsForDetection) {
    debug.reject_reason = "refined_red_cluster_too_small";
    return result;
  }

  const int box_width = max_x - min_x + kScanStride;
  const int box_height = max_y - min_y + kScanStride;
  const int radius = std::max(box_width, box_height) / 2;
  const float aspect_ratio = static_cast<float>(std::max(box_width, box_height)) /
                             static_cast<float>(std::max(1, std::min(box_width, box_height)));
  const float fill_ratio =
      static_cast<float>(best_component_count * kScanStride * kScanStride) /
      static_cast<float>(std::max(1, box_width * box_height));
  debug.candidate_box_width = box_width;
  debug.candidate_box_height = box_height;
  debug.candidate_aspect_ratio = aspect_ratio;
  debug.candidate_fill_ratio = fill_ratio;

  if (radius < kMinLampRadius || radius > kMaxLampRadius || aspect_ratio > kMaxLampAspectRatio ||
      fill_ratio < kMinLampFillRatio) {
    debug.reject_reason = "candidate_shape_rejected";
    return result;
  }

  result.found = true;
  result.min_x = min_x;
  result.min_y = min_y;
  result.max_x = max_x;
  result.max_y = max_y;
  result.center_x = sum_x / best_component_count;
  result.center_y = sum_y / best_component_count;
  result.radius = radius;
  result.pixel_count = best_component_count;

  const int max_allowed_center_y =
      std::max(kMinAllowedMaxLampCenterY,
               static_cast<int>(frame.height * kMaxLampCenterYFraction));
  if (result.center_y > max_allowed_center_y) {
    debug.reject_reason = "candidate_too_low_in_frame";
    result.found = false;
    return result;
  }

  return result;
}

LampDetection detect_red_lamp(const Rgb565Frame& frame, DetectionResult& debug) {
  const CandidateComponent center_override = find_center_override_candidate(frame, debug);
  if (center_override.lamp.found) {
    debug.candidate_box_width = center_override.box_width;
    debug.candidate_box_height = center_override.box_height;
    debug.candidate_aspect_ratio = center_override.aspect_ratio;
    debug.candidate_fill_ratio = center_override.fill_ratio;
    return center_override.lamp;
  }

  return detect_red_lamp_with_grid(frame, debug);
}

}  // namespace

DetectionResult analyze_frame(const Rgb565Frame& frame) {
  DetectionResult result;
  result.reject_reason = "unknown";
  collect_all_debug_red_components(frame, result);
  result.red = detect_red_lamp(frame, result);
  result.red_found = result.red.found;
  if (!result.red_found) {
    return result;
  }

  if (!validate_traffic_light_stack(frame, result.red, result)) {
    if (result.red.pixel_count >= kCenterOverrideMinPixels &&
        result.candidate_box_width <= kCenterOverrideMaxBoxSize &&
        result.candidate_box_height <= kCenterOverrideMaxBoxSize &&
        result.candidate_aspect_ratio <= kCenterOverrideMaxAspectRatio &&
        result.candidate_fill_ratio >= kCenterOverrideMinFillRatio &&
        result.red.center_y >= static_cast<int>(frame.height * kCompactBypassMinYFraction)) {
      result.reject_reason = nullptr;
      return result;
    }

    result.red_found = false;
    result.red.found = false;
    result.reject_reason = "stack_validation_failed";
    return result;
  }

  result.reject_reason = nullptr;
  return result;
}

}  // namespace traffic
