# Arduino ESP32 Serial Camera Sender

This sketch can run in two modes:

- on-device traffic light detection over serial text output
- raw JPEG streaming to the macOS C++ viewer in this repo

## Default board assumption

The included pin map now defaults to **ESP32-S3-EYE**. If your ESP32-S3 camera board is different, edit `camera_pins.h`.

## Arduino IDE setup

1. Install the Arduino ESP32 core.
2. If you are using an OV5640 autofocus module, install the Arduino library `OV5640 Auto Focus for ESP32 Camera`.
2. Open `arduino/esp32_serial_camera_sender/esp32_serial_camera_sender.ino`.
3. Select:
   - Board: your ESP32-S3 camera board, or a generic `ESP32S3 Dev Module` if your variant is not listed
   - Upload Speed: `921600` or `460800`
   - PSRAM: `Enabled` if the option is shown
   - Partition Scheme: default is fine
4. Flash the board.

## Notes

- Do not open Serial Monitor while using the desktop viewer; both compete for the same serial port.
- If the stream is unstable, lower the baud rate and update the desktop viewer command to match.
- If frames are corrupted or slow, reduce `kFrameSize` in `config.h`.
- If the camera does not initialize, your board almost certainly uses a different pin map. Update `camera_pins.h` for the exact module.
- If you have an OV5640 autofocus lens and the image is blurry, install the `OV5640 Auto Focus for ESP32 Camera` library. This sketch will call `focusInit()` and `autoFocusMode()` automatically when the library is present.
- Traffic-light detection mode uses RGB565 frames on-device and prints lines such as `TRAFFIC_LIGHT red_found=1 ... green_on=0 ...` every 500 ms.
- The current detector is heuristic, not Edge Impulse yet. It finds the brightest red lamp, infers the green lamp position below it, and checks whether that ROI turns green.
- When a red light is found, the sketch also packetizes and sends two JPEG snapshots over serial: the full frame and the inferred green-light ROI.
- Optional Wi-Fi debug streaming can be enabled in `config.h`. When enabled, the board serves `/stream`, `/capture`, and `/status` on the local network for debugging.
