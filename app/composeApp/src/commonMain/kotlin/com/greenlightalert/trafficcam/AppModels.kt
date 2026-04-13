package com.greenlightalert.trafficcam

import com.greenlightalert.trafficcam.ble.BleConnectionState
import com.greenlightalert.trafficcam.ble.BlePeripheral
import kotlinx.serialization.Serializable
import kotlinx.serialization.Transient
import kotlin.time.ExperimentalTime

@Serializable
data class DeviceStatus(
    val state: String,
    val summary: String,
    val redFound: Boolean = false,
    val greenOn: Boolean = false,
    val capturedAtEpochMillis: Long,
    val receivedAtEpochMillis: Long = capturedAtEpochMillis,
    val firmwareVersion: String? = null,
)

@Serializable
data class HistoryImage(
    val mimeType: String,
    val width: Int? = null,
    val height: Int? = null,
    val storedFileName: String? = null,
    @Transient val bytes: ByteArray = byteArrayOf(),
)

@Serializable
enum class HistoryKind {
    STATUS,
    IMAGE,
    NOTE,
}

@Serializable
data class HistoryItem(
    val id: String,
    val kind: HistoryKind,
    val title: String,
    val detail: String,
    val createdAtEpochMillis: Long,
    val image: HistoryImage? = null,
)

data class AppUiState(
    val connectionState: BleConnectionState = BleConnectionState.Idle,
    val peripherals: List<BlePeripheral> = emptyList(),
    val scanDebugEntries: List<String> = emptyList(),
    val bleLogEntries: List<String> = emptyList(),
    val pairedDeviceName: String? = null,
    val latestStatus: DeviceStatus? = null,
    val latestImage: HistoryImage? = null,
    val history: List<HistoryItem> = emptyList(),
    val isPaused: Boolean = false,
    val errorMessage: String? = null,
)

@OptIn(ExperimentalTime::class)
fun Long.asTimestampLabel(): String = kotlin.time.Instant.fromEpochMilliseconds(this).toString()
