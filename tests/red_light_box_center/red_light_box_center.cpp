#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include "arduino/esp32_serial_camera_sender/red_light_box_shared.h"
#include "arduino/esp32_serial_camera_sender/shared_detection_config.h"

namespace {

constexpr const char* kRedLightBoxCenterOutDir =
    "/Users/jamesbowler/esp32/tests/red_light_box_center/out";

std::vector<uint8_t> toRgb565Be(const cv::Mat& image) {
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

void unpackRgb565Be(const std::vector<uint8_t>& data, int width, int x, int y, int& r, int& g, int& b) {
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

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: red_light_box_center <image.jpg>\n";
    return 1;
  }

  const std::filesystem::path inputPath(argv[1]);
  const cv::Mat image = cv::imread(inputPath.string(), cv::IMREAD_COLOR);
  if (image.empty()) {
    std::cerr << "failed to read image: " << inputPath << "\n";
    return 1;
  }

  const auto rgb565 = toRgb565Be(image);
  const auto result =
      redbox::evaluateRedClusterInBox(rgb565.data(), image.cols, image.rows, 0, 0, image.cols - 1, image.rows - 1);
  const auto selectedRegion =
      redbox::selectRedRegionFromCluster(0, 0, image.cols - 1, image.rows - 1, result, image.cols, image.rows);
  const int trackX = selectedRegion.centerX;
  const int trackY = selectedRegion.centerY;

  std::cout << "accepted=" << (result.accepted ? 1 : 0)
            << " warm_pixels=" << result.warmPixels
            << " largest_cluster=" << result.largestCluster
            << " cluster_box=(" << result.clusterMinX << "," << result.clusterMinY << ","
            << result.clusterMaxX << "," << result.clusterMaxY << ")"
            << " core_box=(" << result.coreMinX << "," << result.coreMinY << ","
            << result.coreMaxX << "," << result.coreMaxY << ")"
            << " selected_box=(" << selectedRegion.minX << "," << selectedRegion.minY << ","
            << selectedRegion.maxX << "," << selectedRegion.maxY << ")"
            << " brightest=(" << result.brightestX << "," << result.brightestY << ")"
            << " track=(" << trackX << "," << trackY << ")\n";
  std::cout << "annotations: bright_red_pixels=isRedPixel, yellow=cluster_box, green=core_box, "
               "magenta=selected_box, cyan=brightest_pixel, red=tracked_center\n";
  std::cout << "annotation_coords: "
            << "cluster=(" << result.clusterMinX << "," << result.clusterMinY << ")-("
            << result.clusterMaxX << "," << result.clusterMaxY << ") "
            << "core=(" << result.coreMinX << "," << result.coreMinY << ")-("
            << result.coreMaxX << "," << result.coreMaxY << ") "
            << "selected=(" << selectedRegion.minX << "," << selectedRegion.minY << ")-("
            << selectedRegion.maxX << "," << selectedRegion.maxY << ") "
            << "brightest=(" << result.brightestX << "," << result.brightestY << ") "
            << "track=(" << trackX << "," << trackY << ")\n";

  cv::Mat annotated = image.clone();
  int redPixelCount = 0;
  for (int y = 0; y < image.rows; ++y) {
    for (int x = 0; x < image.cols; ++x) {
      int r = 0;
      int g = 0;
      int b = 0;
      unpackRgb565Be(rgb565, image.cols, x, y, r, g, b);
      if (!isRedPixel(r, g, b)) {
        continue;
      }
      ++redPixelCount;
      annotated.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
    }
  }
  cv::rectangle(annotated, cv::Point(result.clusterMinX, result.clusterMinY),
                cv::Point(result.clusterMaxX, result.clusterMaxY), cv::Scalar(0, 255, 255), 1);
  cv::rectangle(annotated, cv::Point(result.coreMinX, result.coreMinY),
                cv::Point(result.coreMaxX, result.coreMaxY), cv::Scalar(0, 255, 0), 1);
  cv::rectangle(annotated, cv::Point(selectedRegion.minX, selectedRegion.minY),
                cv::Point(selectedRegion.maxX, selectedRegion.maxY), cv::Scalar(255, 0, 255), 1);
  cv::circle(annotated, cv::Point(result.brightestX, result.brightestY), 2, cv::Scalar(255, 255, 0), -1);
  cv::circle(annotated, cv::Point(trackX, trackY), 2, cv::Scalar(0, 0, 255), -1);

  const std::filesystem::path outDir = kRedLightBoxCenterOutDir;
  std::filesystem::create_directories(outDir);
  const std::filesystem::path outPath = outDir / inputPath.filename();
  const std::filesystem::path txtPath = outDir / (inputPath.stem().string() + ".txt");
  cv::imwrite(outPath.string(), annotated);
  std::ofstream txt(txtPath);
  txt << "accepted=" << (result.accepted ? 1 : 0) << "\n";
  txt << "is_red_pixel_count=" << redPixelCount << "\n";
  txt << "warm_pixels=" << result.warmPixels << "\n";
  txt << "largest_cluster=" << result.largestCluster << "\n";
  txt << "cluster_min_x=" << result.clusterMinX << "\n";
  txt << "cluster_min_y=" << result.clusterMinY << "\n";
  txt << "cluster_max_x=" << result.clusterMaxX << "\n";
  txt << "cluster_max_y=" << result.clusterMaxY << "\n";
  txt << "cluster_center_x=" << result.clusterCenterX << "\n";
  txt << "cluster_center_y=" << result.clusterCenterY << "\n";
  txt << "core_min_x=" << result.coreMinX << "\n";
  txt << "core_min_y=" << result.coreMinY << "\n";
  txt << "core_max_x=" << result.coreMaxX << "\n";
  txt << "core_max_y=" << result.coreMaxY << "\n";
  txt << "brightest_x=" << result.brightestX << "\n";
  txt << "brightest_y=" << result.brightestY << "\n";
  txt << "brightest_luma=" << result.brightestLuma << "\n";
  txt << "cluster_aspect_ratio=" << result.clusterAspectRatio << "\n";
  txt << "cluster_fill_fraction=" << result.clusterFillFraction << "\n";
  txt << "selected_min_x=" << selectedRegion.minX << "\n";
  txt << "selected_min_y=" << selectedRegion.minY << "\n";
  txt << "selected_max_x=" << selectedRegion.maxX << "\n";
  txt << "selected_max_y=" << selectedRegion.maxY << "\n";
  txt << "track_x=" << trackX << "\n";
  txt << "track_y=" << trackY << "\n";
  std::cout << "annotated=" << outPath.string() << "\n";
  std::cout << "coords=" << txtPath.string() << "\n";
  return 0;
}
