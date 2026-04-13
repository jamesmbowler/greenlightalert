#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Options {
  std::string device;
  std::string windowTitle = "ESP32 Serial Camera";
  speed_t baudConstant = B115200;
  speed_t baudValue = 921600;
  int reconnectDelayMs = 1500;
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
      std::cout
          << "Usage: esp32_serial_camera_viewer --device /dev/cu.usbserial-XXXX "
             "[--baud 921600] [--window-title TITLE]\n"
          << "The ESP32 must stream complete JPEG frames over serial.\n";
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
    if (arg == "--window-title" && i + 1 < argc) {
      options.windowTitle = argv[++i];
      continue;
    }
    if (arg == "--reconnect-delay-ms" && i + 1 < argc) {
      options.reconnectDelayMs = std::stoi(argv[++i]);
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
      throw std::runtime_error("Failed to open serial device: " + options.device +
                               " (" + std::strerror(errno) + ")");
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      close(fd_);
      throw std::runtime_error("tcgetattr failed");
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, options.baudConstant);
    cfsetospeed(&tty, options.baudConstant);
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

bool extractNextJpeg(std::vector<uint8_t>& buffer, std::vector<uint8_t>& jpeg) {
  static constexpr uint8_t kJpegStart[] = {0xFF, 0xD8};
  static constexpr uint8_t kJpegEnd[] = {0xFF, 0xD9};

  const auto start =
      std::search(buffer.begin(), buffer.end(), std::begin(kJpegStart), std::end(kJpegStart));
  if (start == buffer.end()) {
    if (buffer.size() > 1'000'000) {
      buffer.erase(buffer.begin(), buffer.end() - 128'000);
    }
    return false;
  }

  const auto end =
      std::search(start + 2, buffer.end(), std::begin(kJpegEnd), std::end(kJpegEnd));
  if (end == buffer.end()) {
    if (start != buffer.begin()) {
      buffer.erase(buffer.begin(), start);
    }
    return false;
  }

  jpeg.assign(start, end + 2);
  buffer.erase(buffer.begin(), end + 2);
  return true;
}

int run(const Options& options) {
  cv::namedWindow(options.windowTitle, cv::WINDOW_NORMAL);
  std::cout << "Opening serial device: " << options.device << '\n';
  std::cout << "Press q to quit.\n";

  for (;;) {
    try {
      SerialPort port(options);
      std::vector<uint8_t> buffer;
      buffer.reserve(256 * 1024);
      std::vector<uint8_t> jpeg;
      uint8_t chunk[4096];

      for (;;) {
        const ssize_t bytesRead = port.readSome(chunk, sizeof(chunk));
        if (bytesRead < 0) {
          throw std::runtime_error("Serial read failed");
        }
        if (bytesRead == 0) {
          continue;
        }

        buffer.insert(buffer.end(), chunk, chunk + bytesRead);

        while (extractNextJpeg(buffer, jpeg)) {
          cv::Mat frame = cv::imdecode(jpeg, cv::IMREAD_COLOR);
          if (!frame.empty()) {
            cv::imshow(options.windowTitle, frame);
          }

          const int key = cv::waitKey(1) & 0xFF;
          if (key == 'q') {
            return 0;
          }
        }
      }
    } catch (const std::exception& ex) {
      std::cerr << ex.what() << '\n';
      std::cerr << "Retrying in " << options.reconnectDelayMs << " ms.\n";
      std::this_thread::sleep_for(
          std::chrono::milliseconds(options.reconnectDelayMs));
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
