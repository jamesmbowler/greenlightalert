#pragma once

#include <cstdint>

namespace traffic {

constexpr int kMaxDebugCandidates = 32;

struct Rgb565Frame {
  const uint8_t* data = nullptr;
  int width = 0;
  int height = 0;
};

struct LampDetection {
  bool found = false;
  int min_x = 0;
  int min_y = 0;
  int max_x = 0;
  int max_y = 0;
  int center_x = 0;
  int center_y = 0;
  int radius = 0;
  int pixel_count = 0;
};

struct DebugCandidate {
  LampDetection lamp{};
  int avg_r = 0;
  int avg_g = 0;
  int avg_b = 0;
};

struct DetectionResult {
  bool red_found = false;
  LampDetection red{};
  int best_cell_x = -1;
  int best_cell_y = -1;
  int best_cell_count = 0;
  int candidate_box_width = 0;
  int candidate_box_height = 0;
  float candidate_aspect_ratio = 0.0f;
  float candidate_fill_ratio = 0.0f;
  float yellow_dark_fraction = 0.0f;
  float green_dark_fraction = 0.0f;
  float housing_dark_fraction = 0.0f;
  int debug_candidate_count = 0;
  DebugCandidate debug_candidates[kMaxDebugCandidates]{};
  const char* reject_reason = nullptr;
};

DetectionResult analyze_frame(const Rgb565Frame& frame);

}  // namespace traffic
