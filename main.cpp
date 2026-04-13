#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

namespace {

struct Options {
  std::string url = "http://esp32cam.local/stream";
  std::string windowTitle = "ESP32 Camera";
  int reconnectDelayMs = 2000;
};

Options parseArgs(int argc, char** argv) {
  Options options;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: esp32_camera_viewer [stream-url] [--window-title TITLE] "
             "[--reconnect-delay-ms MS]\n"
          << "Default URL: " << options.url << '\n';
      std::exit(0);
    }

    if (arg == "--window-title" && i + 1 < argc) {
      options.windowTitle = argv[++i];
      continue;
    }

    if (arg == "--reconnect-delay-ms" && i + 1 < argc) {
      options.reconnectDelayMs = std::stoi(argv[++i]);
      continue;
    }

    if (!arg.empty() && arg.rfind("--", 0) == 0) {
      throw std::runtime_error("Unknown option: " + arg);
    }

    options.url = arg;
  }

  return options;
}

int run(const Options& options) {
  std::cout << "Opening stream: " << options.url << '\n';
  std::cout << "Press q to quit.\n";

  cv::namedWindow(options.windowTitle, cv::WINDOW_NORMAL);

  for (;;) {
    cv::VideoCapture capture;
    if (!capture.open(options.url, cv::CAP_FFMPEG)) {
      std::cerr << "Failed to open stream. Retrying in "
                << options.reconnectDelayMs << " ms.\n";
      std::this_thread::sleep_for(
          std::chrono::milliseconds(options.reconnectDelayMs));
      continue;
    }

    cv::Mat frame;
    for (;;) {
      if (!capture.read(frame) || frame.empty()) {
        std::cerr << "Stream interrupted. Reconnecting in "
                  << options.reconnectDelayMs << " ms.\n";
        capture.release();
        std::this_thread::sleep_for(
            std::chrono::milliseconds(options.reconnectDelayMs));
        break;
      }

      cv::imshow(options.windowTitle, frame);
      const int key = cv::waitKey(1) & 0xFF;
      if (key == 'q') {
        return 0;
      }
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
