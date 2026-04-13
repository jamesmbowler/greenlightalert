#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "arduino/esp32_serial_camera_sender/traffic_light_detector_shared.h"

namespace {

std::vector<uint8_t> to_rgb565_be(const cv::Mat& image) {
  std::vector<uint8_t> buffer(static_cast<size_t>(image.rows * image.cols * 2));
  size_t out = 0;
  for (int y = 0; y < image.rows; ++y) {
    const auto* row = image.ptr<cv::Vec3b>(y);
    for (int x = 0; x < image.cols; ++x) {
      const uint8_t b = row[x][0];
      const uint8_t g = row[x][1];
      const uint8_t r = row[x][2];
      const uint16_t rgb565 =
          static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
      buffer[out++] = static_cast<uint8_t>(rgb565 >> 8);
      buffer[out++] = static_cast<uint8_t>(rgb565 & 0xFF);
    }
  }
  return buffer;
}

bool parse_expected_red_from_name(const std::string& filename, bool& expected) {
  if (filename.find("redlight_true") != std::string::npos) {
    expected = true;
    return true;
  }
  if (filename.find("redlight_false") != std::string::npos) {
    expected = false;
    return true;
  }
  return false;
}

void annotate_and_save(const std::filesystem::path& out_path, cv::Mat image,
                       const traffic::DetectionResult& result, bool expected) {
  const cv::Scalar green(0, 255, 0);
  const cv::Scalar red(0, 0, 255);
  const cv::Scalar yellow(0, 255, 255);
  const cv::Scalar cyan(255, 255, 0);

  for (int i = 0; i < result.debug_candidate_count; ++i) {
    const auto& candidate = result.debug_candidates[i];
    cv::rectangle(image, cv::Point(candidate.lamp.min_x, candidate.lamp.min_y),
                  cv::Point(candidate.lamp.max_x, candidate.lamp.max_y), yellow, 1);
    cv::circle(image, cv::Point(candidate.lamp.center_x, candidate.lamp.center_y),
               std::max(1, candidate.lamp.radius), cyan, 1);
    std::ostringstream candidate_label;
    candidate_label << i << ":#" << std::hex << std::uppercase
                    << std::setw(2) << std::setfill('0') << candidate.avg_r
                    << std::setw(2) << std::setfill('0') << candidate.avg_g
                    << std::setw(2) << std::setfill('0') << candidate.avg_b;
    cv::putText(image, candidate_label.str(),
                cv::Point(candidate.lamp.min_x, std::max(14, candidate.lamp.min_y - 4)),
                cv::FONT_HERSHEY_SIMPLEX, 0.35, yellow, 1);
  }

  if (result.red_found) {
    cv::rectangle(image, cv::Point(result.red.min_x, result.red.min_y),
                  cv::Point(result.red.max_x, result.red.max_y), green, 2);
    cv::circle(image, cv::Point(result.red.center_x, result.red.center_y),
               std::max(2, result.red.radius), green, 2);
  }

  const std::string label =
      "expected=" + std::to_string(expected ? 1 : 0) + " found=" +
      std::to_string(result.red_found ? 1 : 0);
  cv::putText(image, label, cv::Point(12, 24), cv::FONT_HERSHEY_SIMPLEX, 0.7,
              result.red_found == expected ? green : red, 2);
  cv::imwrite(out_path.string(), image);
}

}  // namespace

int main() {
  const std::filesystem::path test_dir = "tests/find_light/testimages";
  const std::filesystem::path annotated_dir = test_dir / "annotated";
  if (!std::filesystem::exists(test_dir)) {
    std::cerr << "Missing tests/find_light/testimages directory\n";
    return 1;
  }

  std::filesystem::remove_all(annotated_dir);
  std::filesystem::create_directories(annotated_dir);

  int total = 0;
  int failures = 0;

  for (const auto& entry : std::filesystem::directory_iterator(test_dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }

    const auto path = entry.path();
    bool expected = false;
    if (!parse_expected_red_from_name(path.filename().string(), expected)) {
      continue;
    }

    const auto image = cv::imread(path.string(), cv::IMREAD_COLOR);
    if (image.empty()) {
      std::cerr << "Failed to read " << path << '\n';
      ++failures;
      ++total;
      continue;
    }

    const auto rgb565 = to_rgb565_be(image);
    const traffic::Rgb565Frame frame{rgb565.data(), image.cols, image.rows};
    const auto result = traffic::analyze_frame(frame);
    const bool passed = (result.red_found == expected);
    annotate_and_save(annotated_dir / path.filename(), image.clone(), result, expected);

    std::cout << path.filename().string() << " expected_red=" << (expected ? 1 : 0)
              << " found_red=" << (result.red_found ? 1 : 0);
    if (result.red_found) {
      std::cout << " center=(" << result.red.center_x << "," << result.red.center_y << ")"
                << " box=(" << result.red.min_x << "," << result.red.min_y << ","
                << result.red.max_x << "," << result.red.max_y << ")"
                << " dark(y=" << result.yellow_dark_fraction
                << ",g=" << result.green_dark_fraction
                << ",housing=" << result.housing_dark_fraction << ")";
    } else {
      std::cout << " reject_reason=" << (result.reject_reason ? result.reject_reason : "none")
                << " best_cell=(" << result.best_cell_x << "," << result.best_cell_y << ")"
                << " best_cell_count=" << result.best_cell_count
                << " box=(" << result.candidate_box_width << "x" << result.candidate_box_height
                << ")"
                << " aspect=" << result.candidate_aspect_ratio
                << " fill=" << result.candidate_fill_ratio
                << " dark(y=" << result.yellow_dark_fraction
                << ",g=" << result.green_dark_fraction
                << ",housing=" << result.housing_dark_fraction << ")";
    }
    if (result.debug_candidate_count > 0) {
      std::cout << " candidates=";
      for (int i = 0; i < result.debug_candidate_count; ++i) {
        const auto& candidate = result.debug_candidates[i];
        std::cout << "[" << i << " center=(" << candidate.lamp.center_x << ","
                  << candidate.lamp.center_y << ") rgb=#" << std::hex << std::uppercase
                  << std::setw(2) << std::setfill('0') << candidate.avg_r
                  << std::setw(2) << std::setfill('0') << candidate.avg_g
                  << std::setw(2) << std::setfill('0') << candidate.avg_b
                  << std::dec << " px=" << candidate.lamp.pixel_count << "]";
      }
    }
    std::cout << '\n';

    if (!passed) {
      ++failures;
    }
    ++total;
  }

  if (total == 0) {
    throw std::runtime_error("No test images matched redlight_true/redlight_false naming");
  }

  std::cout << "Summary: " << (total - failures) << "/" << total << " passed\n";
  if (failures != 0) {
    std::cerr << "ASSERTION FAILED: red-light detection expectations were not met\n";
    return 1;
  }
  return 0;
}
