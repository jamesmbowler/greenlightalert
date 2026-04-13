#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

struct Options {
  std::string device;
  std::filesystem::path outputDir = "trafficimages";
  speed_t baudValue = 921600;
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

constexpr char kMagic[] = {'T', 'L', 'I', 'M'};
constexpr uint8_t kVersion = 1;
constexpr uint8_t kImageTypeRedFrame = 1;
constexpr uint8_t kImageTypeGreenRoi = 2;
constexpr uint8_t kImageTypePeriodicFrame = 3;
constexpr uint8_t kImageTypeRedFrameBoxes = 4;
constexpr uint8_t kImageTypeRedFrameEi = 5;
constexpr uint8_t kImageTypeEiModelInput = 6;
constexpr uint8_t kImageTypeEiCropRoi = 7;
constexpr uint8_t kImageTypeGreenLight = 8;
constexpr uint8_t kImageTypeGreenLightSearch = 9;
constexpr uint8_t kImageTypeSceneChanged = 10;
constexpr uint8_t kImageTypeRedLightBox = 11;
constexpr uint8_t kImageTypeRawEiOutput = 12;
constexpr uint32_t kMaxPayloadSize = 2 * 1024 * 1024;

struct PendingRedFrame {
  std::filesystem::path imagePath;
  std::filesystem::path annotatedPath;
  std::filesystem::path bboxPath;
  std::filesystem::path eiModelInputPath;
  std::filesystem::path eiModelInputAnnotatedPath;
  bool frameAnnotated = false;
  bool eiAnnotated = false;
};

speed_t parseBaudValue(const std::string& value) {
  const int baud = std::stoi(value);
  if (baud <= 0) {
    throw std::runtime_error("Baud rate must be positive");
  }
  return static_cast<speed_t>(baud);
}

Options parseArgs(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: serial_saver --device /dev/cu.usbmodem1101 [--baud 921600] "
                   "[--output-dir trafficimages]\n";
      std::exit(0);
    }
    if (arg == "--device" && i + 1 < argc) {
      options.device = argv[++i];
      continue;
    }
    if (arg == "--baud" && i + 1 < argc) {
      options.baudValue = parseBaudValue(argv[++i]);
      continue;
    }
    if (arg == "--output-dir" && i + 1 < argc) {
      options.outputDir = argv[++i];
      continue;
    }
    throw std::runtime_error("Unknown or incomplete option: " + arg);
  }
  if (options.device.empty()) {
    throw std::runtime_error("Missing required argument: --device /dev/cu.*");
  }
  return options;
}

class SerialPort {
 public:
  explicit SerialPort(const Options& options) {
    fd_ = open(options.device.c_str(), O_RDONLY | O_NOCTTY);
    if (fd_ < 0) {
      throw std::runtime_error("Failed to open serial device: " + options.device + " (" +
                               std::strerror(errno) + ")");
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      close(fd_);
      throw std::runtime_error("tcgetattr failed");
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      close(fd_);
      throw std::runtime_error("tcsetattr failed");
    }

    speed_t baudValue = options.baudValue;
    if (ioctl(fd_, IOSSIOSPEED, &baudValue) == -1) {
      close(fd_);
      throw std::runtime_error("Failed to set custom baud rate");
    }
  }

  ~SerialPort() {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  ssize_t readSome(uint8_t* buffer, size_t size) { return ::read(fd_, buffer, size); }

 private:
  int fd_ = -1;
};

std::string imageTypeName(uint8_t type) {
  if (type == kImageTypeRedFrame) return "red_frame";
  if (type == kImageTypeGreenRoi) return "green_roi";
  if (type == kImageTypePeriodicFrame) return "periodic_frame";
  if (type == kImageTypeRedFrameBoxes) return "red_frame_boxes";
  if (type == kImageTypeRedFrameEi) return "red_frame_ei";
  if (type == kImageTypeEiModelInput) return "ei_model_input";
  if (type == kImageTypeEiCropRoi) return "ei_crop_roi";
  if (type == kImageTypeGreenLight) return "green_light";
  if (type == kImageTypeGreenLightSearch) return "green_light_search";
  if (type == kImageTypeSceneChanged) return "scene_changed";
  if (type == kImageTypeRedLightBox) return "red_light_box";
  if (type == kImageTypeRawEiOutput) return "raw_ei_output";
  return "unknown";
}

std::filesystem::path buildPacketPath(const Options& options, uint8_t imageType, uint32_t sequence,
                                      int64_t timestamp, const char* extension) {
  return options.outputDir /
         (imageTypeName(imageType) + "_" + std::to_string(sequence) + "_" +
          std::to_string(timestamp) + extension);
}

std::map<std::string, std::string> parseKeyValueText(const std::vector<uint8_t>& payload) {
  std::map<std::string, std::string> values;
  const std::string text(payload.begin(), payload.end());
  size_t start = 0;
  while (start < text.size()) {
    const size_t end = text.find('\n', start);
    const std::string line =
        text.substr(start, end == std::string::npos ? std::string::npos : end - start);
    const size_t equals = line.find('=');
    if (equals != std::string::npos) {
      values.emplace(line.substr(0, equals), line.substr(equals + 1));
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  return values;
}

void annotateRedFrame(const std::filesystem::path& imagePath,
                      const std::filesystem::path& annotatedPath,
                      const std::map<std::string, std::string>& bboxInfo) {
  const auto foundIt = bboxInfo.find("red_found");
  if (foundIt == bboxInfo.end() || foundIt->second != "1") {
    return;
  }

  cv::Mat image = cv::imread(imagePath.string(), cv::IMREAD_COLOR);
  if (image.empty()) {
    throw std::runtime_error("Failed to load image for annotation: " + imagePath.string());
  }

  const int boxesCount = std::stoi(bboxInfo.count("boxes_count") ? bboxInfo.at("boxes_count") : "0");
  for (int i = 0; i < boxesCount; ++i) {
    const std::string prefix = "box_" + std::to_string(i) + "_";
    const int xMin = std::stoi(bboxInfo.at(prefix + "x_min"));
    const int yMin = std::stoi(bboxInfo.at(prefix + "y_min"));
    const int xMax = std::stoi(bboxInfo.at(prefix + "x_max"));
    const int yMax = std::stoi(bboxInfo.at(prefix + "y_max"));
    const float confidence = std::stof(bboxInfo.at(prefix + "confidence"));
    const std::string label = bboxInfo.at(prefix + "label") +
                              " " + std::to_string(static_cast<int>(confidence * 100)) + "%";
    const int centerX = (xMin + xMax) / 2;
    const int centerY = (yMin + yMax) / 2;
    cv::rectangle(image, cv::Point(xMin, yMin), cv::Point(xMax, yMax), cv::Scalar(0, 0, 255), 2);
    cv::circle(image, cv::Point(centerX, centerY), 4, cv::Scalar(0, 255, 255), cv::FILLED);
    cv::putText(image, label, cv::Point(xMin, std::max(18, yMin - 6)), cv::FONT_HERSHEY_SIMPLEX,
                0.6, cv::Scalar(0, 255, 255), 2);
  }

  const auto centerXIt = bboxInfo.find("center_x");
  const auto centerYIt = bboxInfo.find("center_y");
  if (centerXIt != bboxInfo.end() && centerYIt != bboxInfo.end()) {
    const int centerX = std::clamp(std::stoi(centerXIt->second), 0, image.cols - 1);
    const int centerY = std::clamp(std::stoi(centerYIt->second), 0, image.rows - 1);
    image.at<cv::Vec3b>(centerY, centerX) = cv::Vec3b(0, 0, 0);
  }

  const auto bboxXMinIt = bboxInfo.find("bbox_x_min");
  const auto bboxYMinIt = bboxInfo.find("bbox_y_min");
  const auto bboxXMaxIt = bboxInfo.find("bbox_x_max");
  const auto bboxYMaxIt = bboxInfo.find("bbox_y_max");
  if (bboxXMinIt != bboxInfo.end() && bboxYMinIt != bboxInfo.end() &&
      bboxXMaxIt != bboxInfo.end() && bboxYMaxIt != bboxInfo.end()) {
    const int bboxXMin = std::clamp(std::stoi(bboxXMinIt->second), 0, image.cols - 1);
    const int bboxYMin = std::clamp(std::stoi(bboxYMinIt->second), 0, image.rows - 1);
    const int bboxXMax = std::clamp(std::stoi(bboxXMaxIt->second), bboxXMin, image.cols - 1);
    const int bboxYMax = std::clamp(std::stoi(bboxYMaxIt->second), bboxYMin, image.rows - 1);
    cv::rectangle(image, cv::Point(bboxXMin, bboxYMin), cv::Point(bboxXMax, bboxYMax),
                  cv::Scalar(255, 0, 255), 2);
    cv::putText(image, "green-watch red box",
                cv::Point(bboxXMin, std::min(image.rows - 6, bboxYMax + 18)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
  }

  if (!cv::imwrite(annotatedPath.string(), image)) {
    throw std::runtime_error("Failed to save annotated image: " + annotatedPath.string());
  }
}

void annotateEiModelInput(const std::filesystem::path& imagePath,
                          const std::filesystem::path& annotatedPath,
                          const std::map<std::string, std::string>& bboxInfo) {
  cv::Mat image = cv::imread(imagePath.string(), cv::IMREAD_COLOR);
  if (image.empty()) {
    throw std::runtime_error("Failed to load EI image for annotation: " + imagePath.string());
  }

  const int boxesCount = std::stoi(bboxInfo.count("boxes_count") ? bboxInfo.at("boxes_count") : "0");
  for (int i = 0; i < boxesCount; ++i) {
    const std::string prefix = "box_" + std::to_string(i) + "_";
    const auto rawXIt = bboxInfo.find(prefix + "raw_x");
    const auto rawYIt = bboxInfo.find(prefix + "raw_y");
    const auto rawWidthIt = bboxInfo.find(prefix + "raw_width");
    const auto rawHeightIt = bboxInfo.find(prefix + "raw_height");
    if (rawXIt == bboxInfo.end() || rawYIt == bboxInfo.end() ||
        rawWidthIt == bboxInfo.end() || rawHeightIt == bboxInfo.end()) {
      continue;
    }

    const int rawX = std::stoi(rawXIt->second);
    const int rawY = std::stoi(rawYIt->second);
    const int rawWidth = std::stoi(rawWidthIt->second);
    const int rawHeight = std::stoi(rawHeightIt->second);
    const int xMin = std::max(0, rawX);
    const int yMin = std::max(0, rawY);
    const int xMax = std::min(image.cols - 1, rawX + rawWidth - 1);
    const int yMax = std::min(image.rows - 1, rawY + rawHeight - 1);
    if (xMin > xMax || yMin > yMax) {
      continue;
    }

    const float confidence = std::stof(bboxInfo.at(prefix + "confidence"));
    const std::string label = bboxInfo.at(prefix + "label") +
                              " " + std::to_string(static_cast<int>(confidence * 100)) + "%";
    cv::rectangle(image, cv::Point(xMin, yMin), cv::Point(xMax, yMax), cv::Scalar(0, 0, 255), 2);
    cv::putText(image, label, cv::Point(xMin, std::max(18, yMin - 6)), cv::FONT_HERSHEY_SIMPLEX,
                0.5, cv::Scalar(0, 255, 255), 1);
  }

  if (!cv::imwrite(annotatedPath.string(), image)) {
    throw std::runtime_error("Failed to save annotated EI image: " + annotatedPath.string());
  }
}

void saveRawRgbPpm(const std::filesystem::path& path, uint16_t width, uint16_t height,
                   const std::vector<uint8_t>& payload) {
  const size_t expectedSize = static_cast<size_t>(width) * static_cast<size_t>(height) * 3;
  if (payload.size() != expectedSize) {
    throw std::runtime_error("Unexpected raw RGB payload size for " + path.string());
  }

  FILE* file = std::fopen(path.c_str(), "wb");
  if (file == nullptr) {
    throw std::runtime_error("Failed to create file: " + path.string());
  }

  const std::string header =
      "P6\n" + std::to_string(width) + " " + std::to_string(height) + "\n255\n";
  const size_t headerWritten = std::fwrite(header.data(), 1, header.size(), file);
  const size_t payloadWritten = std::fwrite(payload.data(), 1, payload.size(), file);
  std::fclose(file);

  if (headerWritten != header.size() || payloadWritten != payload.size()) {
    throw std::runtime_error("Short write when saving: " + path.string());
  }
}

bool tryExtractPacket(std::vector<uint8_t>& buffer, ImagePacketHeader& header,
                      std::vector<uint8_t>& payload) {
  const auto searchStart = buffer.begin();
  const auto searchEnd = buffer.end();
  const auto magicPos = std::search(searchStart, searchEnd, std::begin(kMagic), std::end(kMagic));
  if (magicPos == buffer.end()) {
    if (buffer.size() > 4096) {
      buffer.erase(buffer.begin(), buffer.end() - 4);
    }
    return false;
  }

  if (magicPos != buffer.begin()) {
    buffer.erase(buffer.begin(), magicPos);
  }

  if (buffer.size() < sizeof(ImagePacketHeader)) {
    return false;
  }

  std::memcpy(&header, buffer.data(), sizeof(ImagePacketHeader));
  if (std::memcmp(header.magic, kMagic, sizeof(kMagic)) != 0 || header.version != kVersion ||
      header.payloadSize > kMaxPayloadSize) {
    buffer.erase(buffer.begin());
    return false;
  }

  const size_t packetSize = sizeof(ImagePacketHeader) + header.payloadSize;
  if (buffer.size() < packetSize) {
    return false;
  }

  payload.assign(buffer.begin() + sizeof(ImagePacketHeader), buffer.begin() + packetSize);
  buffer.erase(buffer.begin(), buffer.begin() + packetSize);
  return true;
}

void savePacket(const Options& options, const ImagePacketHeader& header,
                const std::vector<uint8_t>& payload,
                std::map<uint32_t, PendingRedFrame>& pendingRedFrames) {
  std::filesystem::create_directories(options.outputDir);
  const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();

  if (header.imageType == kImageTypeRedFrameBoxes) {
    const auto path = buildPacketPath(options, header.imageType, header.sequence, timestamp, ".txt");
    FILE* file = std::fopen(path.c_str(), "wb");
    if (file == nullptr) {
      throw std::runtime_error("Failed to create file: " + path.string());
    }
    const size_t written = std::fwrite(payload.data(), 1, payload.size(), file);
    std::fclose(file);
    if (written != payload.size()) {
      throw std::runtime_error("Short write when saving: " + path.string());
    }

    const auto bboxInfo = parseKeyValueText(payload);
    auto& pending = pendingRedFrames[header.sequence];
    pending.bboxPath = path;
    auto pendingIt = pendingRedFrames.find(header.sequence);
    if (pendingIt != pendingRedFrames.end()) {
      bool annotatedAnything = false;
      if (!pendingIt->second.imagePath.empty() && !pendingIt->second.frameAnnotated) {
        annotateRedFrame(pendingIt->second.imagePath, pendingIt->second.annotatedPath, bboxInfo);
        std::cout << "Saved " << pendingIt->second.annotatedPath << " (annotated)\n";
        pendingIt->second.frameAnnotated = true;
        annotatedAnything = true;
      }
      if (!pendingIt->second.eiModelInputPath.empty() && !pendingIt->second.eiAnnotated) {
        annotateEiModelInput(pendingIt->second.eiModelInputPath,
                             pendingIt->second.eiModelInputAnnotatedPath, bboxInfo);
        std::cout << "Saved " << pendingIt->second.eiModelInputAnnotatedPath
                  << " (ei annotated)\n";
        pendingIt->second.eiAnnotated = true;
        annotatedAnything = true;
      }
      if (pendingIt->second.frameAnnotated && pendingIt->second.eiAnnotated) {
        pendingIt->second.bboxPath.clear();
      }
      if (annotatedAnything && pendingIt->second.bboxPath.empty()) {
        pendingRedFrames.erase(pendingIt);
      }
    }

    std::cout << "Saved " << path << " (bbox info)\n";
    return;
  }

  if (header.imageType == kImageTypeRawEiOutput) {
    const auto path = buildPacketPath(options, header.imageType, header.sequence, timestamp, ".txt");
    FILE* file = std::fopen(path.c_str(), "wb");
    if (file == nullptr) {
      throw std::runtime_error("Failed to create file: " + path.string());
    }
    const size_t written = std::fwrite(payload.data(), 1, payload.size(), file);
    std::fclose(file);
    if (written != payload.size()) {
      throw std::runtime_error("Short write when saving: " + path.string());
    }
    std::cout << "Saved " << path << " (raw EI output)\n";
    return;
  }

  if (header.imageType == kImageTypeEiModelInput) {
    const auto path = buildPacketPath(options, header.imageType, header.sequence, timestamp, ".ppm");
    saveRawRgbPpm(path, header.width, header.height, payload);
    std::cout << "Saved " << path << " (" << header.width << "x" << header.height
              << ", raw RGB)\n";
    auto& pending = pendingRedFrames[header.sequence];
    pending.eiModelInputPath = path;
    pending.eiModelInputAnnotatedPath =
        path.parent_path() / (path.stem().string() + "_annotated.png");
    if (!pending.bboxPath.empty()) {
      const auto bboxPayload = [&]() {
        FILE* file = std::fopen(pending.bboxPath.c_str(), "rb");
        if (file == nullptr) {
          throw std::runtime_error("Failed to open bbox file: " + pending.bboxPath.string());
        }
        std::vector<uint8_t> data;
        std::fseek(file, 0, SEEK_END);
        const long size = std::ftell(file);
        std::rewind(file);
        if (size > 0) {
          data.resize(static_cast<size_t>(size));
          const size_t read = std::fread(data.data(), 1, data.size(), file);
          std::fclose(file);
          if (read != data.size()) {
            throw std::runtime_error("Short read from bbox file: " + pending.bboxPath.string());
          }
        } else {
          std::fclose(file);
        }
        return data;
      }();
      const auto bboxInfo = parseKeyValueText(bboxPayload);
      if (!pending.eiAnnotated) {
        annotateEiModelInput(path, pending.eiModelInputAnnotatedPath, bboxInfo);
        std::cout << "Saved " << pending.eiModelInputAnnotatedPath << " (ei annotated)\n";
        pending.eiAnnotated = true;
      }
      if (pending.frameAnnotated && pending.eiAnnotated) {
        pending.bboxPath.clear();
      }
      if (pending.bboxPath.empty() && pending.imagePath.empty()) {
        pendingRedFrames.erase(header.sequence);
      }
    }
    return;
  }

  const auto path = buildPacketPath(options, header.imageType, header.sequence, timestamp, ".jpg");
  FILE* file = std::fopen(path.c_str(), "wb");
  if (file == nullptr) {
    throw std::runtime_error("Failed to create file: " + path.string());
  }
  const size_t written = std::fwrite(payload.data(), 1, payload.size(), file);
  std::fclose(file);
  if (written != payload.size()) {
    throw std::runtime_error("Short write when saving: " + path.string());
  }
  std::cout << "Saved " << path << " (" << header.width << "x" << header.height << ")\n";

  if (header.imageType == kImageTypeRedFrame || header.imageType == kImageTypeRedFrameEi) {
    auto& pending = pendingRedFrames[header.sequence];
    pending.imagePath = path;
    pending.annotatedPath =
        path.parent_path() / (path.stem().string() + "_annotated" + path.extension().string());
    if (!pending.bboxPath.empty()) {
      FILE* file = std::fopen(pending.bboxPath.c_str(), "rb");
      if (file != nullptr) {
        std::vector<uint8_t> data;
        std::fseek(file, 0, SEEK_END);
        const long size = std::ftell(file);
        std::rewind(file);
        if (size > 0) {
          data.resize(static_cast<size_t>(size));
          const size_t read = std::fread(data.data(), 1, data.size(), file);
          std::fclose(file);
          if (read != data.size()) {
            throw std::runtime_error("Short read from bbox file: " + pending.bboxPath.string());
          }
        } else {
          std::fclose(file);
        }
        if (!pending.frameAnnotated) {
          const auto bboxInfo = parseKeyValueText(data);
          annotateRedFrame(path, pending.annotatedPath, bboxInfo);
          std::cout << "Saved " << pending.annotatedPath << " (annotated)\n";
          pending.frameAnnotated = true;
          if (pending.eiAnnotated) {
            pending.bboxPath.clear();
          }
        }
      }
    }
  }
}

int run(const Options& options) {
  SerialPort port(options);
  std::vector<uint8_t> buffer;
  std::map<uint32_t, PendingRedFrame> pendingRedFrames;
  buffer.reserve(128 * 1024);
  uint8_t chunk[4096];

  std::cout << "Saving packetized images from " << options.device << " into "
            << options.outputDir << '\n';

  for (;;) {
    const ssize_t bytesRead = port.readSome(chunk, sizeof(chunk));
    if (bytesRead < 0) {
      throw std::runtime_error("Serial read failed");
    }
    if (bytesRead == 0) {
      continue;
    }

    buffer.insert(buffer.end(), chunk, chunk + bytesRead);

    for (;;) {
      ImagePacketHeader header{};
      std::vector<uint8_t> payload;
      if (!tryExtractPacket(buffer, header, payload)) {
        break;
      }
      savePacket(options, header, payload, pendingRedFrames);
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parseArgs(argc, argv);
    return run(options);
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << '\n';
    return 1;
  }
}
