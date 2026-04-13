package com.greenlightalert.trafficcam.ble

import com.greenlightalert.trafficcam.DeviceStatus
import kotlinx.serialization.Serializable
import kotlin.time.Clock
import kotlin.time.ExperimentalTime

data class BlePeripheral(
    val id: String,
    val name: String?,
    val rssi: Int,
)

enum class BleConnectionState {
    Idle,
    Scanning,
    Connecting,
    Paired,
    Monitoring,
    Error,
}

@Serializable
data class BleStatusPayload(
    val s: Int,
    val r: Int = 0,
    val g: Int = 0,
    val firmwareVersion: String? = null,
) {
    @OptIn(ExperimentalTime::class)
    fun toDeviceStatus(): DeviceStatus {
        val receivedAt = Clock.System.now().toEpochMilliseconds()
        return DeviceStatus(
            state = statusTitle(s),
            summary = statusSummary(s),
            redFound = r != 0,
            greenOn = g != 0,
            capturedAtEpochMillis = receivedAt,
            receivedAtEpochMillis = receivedAt,
            firmwareVersion = firmwareVersion,
        )
    }

    private fun statusTitle(code: Int): String = when (code) {
        0 -> "Monitoring"
        1 -> "Red detected"
        2 -> "Watching green"
        3 -> "Green observed"
        4 -> "Scene changed"
        5 -> "Sending image"
        6 -> "Paused"
        else -> "Booted"
    }

    private fun statusSummary(code: Int): String = when (code) {
        0 -> "Scanning for a red light."
        1 -> "Locked onto a red signal and monitoring for a change."
        2 -> "Red signal locked, watching for the scene to change."
        3 -> "Red lamp cleared and the signal changed."
        4 -> "Tracking target moved or the scene changed."
        5 -> "Sending the latest requested frame over BLE."
        6 -> "Paused. Detection and image capture are suspended."
        else -> "Camera initialized and awaiting BLE connection."
    }
}

data class BleImageMetadata(
    val imageId: String,
    val reason: String,
    val mimeType: String = "image/jpeg",
    val width: Int? = null,
    val height: Int? = null,
    val totalBytes: Int,
    val capturedAtEpochMillis: Long,
    val deviceCapturedAtMs: Long = capturedAtEpochMillis,
    val receivedAtEpochMillis: Long = capturedAtEpochMillis,
    val chunkPayloadSize: Int,
) {
    fun totalChunks(): Int = (totalBytes + chunkPayloadSize - 1) / chunkPayloadSize

    fun windowChunkCount(): Int = maxOf(1, (totalChunks() + WINDOW_PARTS - 1) / WINDOW_PARTS)
}

sealed interface BleIncomingEvent {
    data class StatusReceived(val status: DeviceStatus) : BleIncomingEvent
    data class ImageReceived(val metadata: BleImageMetadata, val bytes: ByteArray) : BleIncomingEvent
    data class LogMessage(val message: String) : BleIncomingEvent
    data class ScanObservation(val message: String) : BleIncomingEvent
}

@OptIn(ExperimentalTime::class)
fun parseBleImageMetadata(value: ByteArray): BleImageMetadata {
    require(value.size >= 15) { "Expected 15-byte image metadata header, got ${value.size}" }
    val reasonCode = value[0].toInt() and 0xFF
    val width = readLeUInt16(value, 1)
    val height = readLeUInt16(value, 3)
    val totalBytes = readLeInt32(value, 5)
    val deviceCapturedAtMs = readLeUInt32(value, 9)
    val receivedAtEpochMillis = Clock.System.now().toEpochMilliseconds()
    val chunkPayloadSize = readLeUInt16(value, 13)
    return BleImageMetadata(
        imageId = "$receivedAtEpochMillis-$reasonCode",
        reason = imageReasonLabel(reasonCode),
        width = width,
        height = height,
        totalBytes = totalBytes,
        capturedAtEpochMillis = receivedAtEpochMillis,
        deviceCapturedAtMs = deviceCapturedAtMs,
        receivedAtEpochMillis = receivedAtEpochMillis,
        chunkPayloadSize = chunkPayloadSize,
    )
}

private fun imageReasonLabel(code: Int): String = when (code) {
    1 -> "red_light_detected"
    2 -> "green_light_detected"
    3 -> "manual_request"
    else -> "unknown"
}

private fun readLeUInt16(bytes: ByteArray, offset: Int): Int =
    (bytes[offset].toInt() and 0xFF) or
        ((bytes[offset + 1].toInt() and 0xFF) shl 8)

private fun readLeInt32(bytes: ByteArray, offset: Int): Int =
    (bytes[offset].toInt() and 0xFF) or
        ((bytes[offset + 1].toInt() and 0xFF) shl 8) or
        ((bytes[offset + 2].toInt() and 0xFF) shl 16) or
        ((bytes[offset + 3].toInt() and 0xFF) shl 24)

private fun readLeUInt32(bytes: ByteArray, offset: Int): Long =
    readLeInt32(bytes, offset).toLong() and 0xFFFF_FFFFL

private const val WINDOW_PARTS = 4
