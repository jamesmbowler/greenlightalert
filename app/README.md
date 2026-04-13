# TrafficCam KMP App

This folder contains a basic Kotlin Multiplatform mobile app scaffold for the ESP32 traffic camera workflow:

- scan and pair to the ESP32 over BLE
- show the latest detector status
- receive a red-light image over BLE
- store and show a timeline of status changes and received images

## Structure

- `composeApp`: shared Compose UI plus platform-specific BLE and history storage
- `iosMain`: iOS entry point and a stub BLE client for now
- `androidMain`: Android BLE implementation using GATT notifications

## BLE contract expected from the ESP32

Service UUID:

```text
7A1C0000-0C4E-4F67-8D80-61A2E5B10000
```

Characteristics:

- `7A1C0001-0C4E-4F67-8D80-61A2E5B10000`
  Status JSON, `notify` and optional `read`
- `7A1C0002-0C4E-4F67-8D80-61A2E5B10000`
  Image metadata JSON, `notify`
- `7A1C0003-0C4E-4F67-8D80-61A2E5B10000`
  Raw image chunk bytes, `notify`
- `7A1C0004-0C4E-4F67-8D80-61A2E5B10000`
  Control JSON, `write`

Status payload example:

```json
{
  "state": "Red detected",
  "summary": "Locked on red lamp, transmitting JPEG",
  "redFound": true,
  "greenOn": false,
  "capturedAtEpochMillis": 1774202775000,
  "firmwareVersion": "trafficcam-ble-0.1"
}
```

Image metadata payload example:

```json
{
  "imageId": "2026-03-22T19:52:55.000Z",
  "reason": "red_light_detected",
  "mimeType": "image/jpeg",
  "width": 320,
  "height": 240,
  "totalBytes": 18234,
  "capturedAtEpochMillis": 1774202775000
}
```

Image transfer flow:

1. Notify `status` when the camera state changes.
2. Notify `image metadata` once before the corresponding JPEG bytes.
3. Stream the JPEG over repeated `image data` notifications until `totalBytes` are sent.
4. The Android app assembles the chunks into one image and stores it in history.

Control payload supported by the app:

```json
{"command":"request_latest_image"}
```

## Notes

- Android BLE is implemented as a first pass and should work once runtime permissions are granted and the ESP32 advertises the service above.
- iOS is included as a KMP target and entry point, but the BLE client there is currently a stub so the UI can run while CoreBluetooth wiring is added later.
- History persistence is file-backed on both platforms as JSON.

## Build

If Gradle is working locally:

```bash
cd /Users/jamesbowler/esp32/app
./gradlew :composeApp:assembleDebug
```
