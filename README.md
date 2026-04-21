# ESP32 Project for detecting a Light change from Red to Green

# Purpose
- Check out the [video](https://youtube.com/shorts/Ix7TiHLy53g) of it in action 
- I don't like having to watch a red light until it turns green, so I made the green light alert. As of now, it sends me an alert over bluetooth when the light turns green. It's using Edge Impulse.

#Hardware needed ( so far )
- [esp32s3 sense](https://www.amazon.com/Seeed-Studio-XIAO-ESP32-Sense/dp/B0C69FFVHH) 
- [A basic on/off switch](https://www.amazon.com/dp/B0DLBD4X2B)
- [A liPo battery like this](https://www.amazon.com/dp/B08FD39Y5R)
- An android phone (for now)

This repo contains:

- `esp32_serial_camera_viewer`: native macOS C++ viewer for JPEG frames over serial
- `esp32_camera_viewer`: native macOS C++ viewer for HTTP MJPEG streams
- `serial_saver`: native macOS C++ serial packet receiver that saves images into `trafficimages/`
- `arduino/esp32_serial_camera_sender`: Arduino sketch that either sends camera JPEG frames over serial or runs on-device traffic-light detection

## Desktop build

Requirements:

- CMake
- OpenCV

Build:

```bash
cmake -S . -B build
cmake --build build
```

## Arduino project

Open this sketch in Arduino IDE:

```text
arduino/esp32_serial_camera_sender/esp32_serial_camera_sender.ino
```

Default target board profile is **Seeed Studio XIAO ESP32S3 Sense**. If your hardware is different, edit:

```text
arduino/esp32_serial_camera_sender/camera_pins.h
```

Key camera and serial settings live in:

```text
arduino/esp32_serial_camera_sender/config.h
```

If you are using an OV5640 autofocus module, also install the Arduino library `OV5640 Auto Focus for ESP32 Camera`. The sketch enables autofocus automatically when that library is available.

Traffic-light detection currently runs entirely on the ESP32-S3 using RGB565 frames and heuristic color/blob logic. It reports the red lamp location plus an inferred green-lamp ROI every 500 ms.

Optional debug MJPEG streaming is available over Wi-Fi. Enable it in [config.h](/Users/jamesbowler/esp32/arduino/esp32_serial_camera_sender/config.h) by setting:

- `kEnableDebugMjpegStream = true`
- `kWifiSsid`
- `kWifiPassword`

Debug endpoints:

- `/stream` for MJPEG
- `/capture` for a single JPEG
- `/status` for JSON detector status

This is intended for debugging only and can be disabled cleanly without affecting the main detector flow.

When the ESP32 finds a red light, it also sends two packetized JPEG images over serial:

- the full frame where the red light was found
- the inferred green-light ROI

Build and run the image saver:

```bash
cmake -S . -B build
cmake --build build --target serial_saver
./build/serial_saver --device /dev/cu.usbmodem1101
```

Saved images are written to:

```text
trafficimages/
```

## Serial run

Find the serial device:

```bash
ls /dev/cu.*
```

Run the macOS viewer:

```bash
./build/esp32_serial_camera_viewer --device /dev/cu.usbmodem1101 --baud 921600
```

Press `q` to quit.

Do not open Serial Monitor at the same time.

If camera init fails on your ESP32-S3 board, the usual cause is a mismatched camera pin map rather than a desktop-side issue.
