package com.greenlightalert.trafficcam

import com.greenlightalert.trafficcam.ble.BleClient
import com.greenlightalert.trafficcam.ble.BleIncomingEvent
import com.greenlightalert.trafficcam.ble.BlePeripheral
import com.greenlightalert.trafficcam.history.HistoryStore
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.collect
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.Job

class TrafficCamPresenter(
    private val bleClient: BleClient,
    private val historyStore: HistoryStore,
    private val alertPlayer: AlertPlayer,
) {
    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Main)
    private val _uiState = MutableStateFlow(AppUiState())
    val uiState: StateFlow<AppUiState> = _uiState.asStateFlow()
    private var scanTimeoutJob: Job? = null
    private var historySequence: Long = 0

    init {
        scope.launch {
            val existing = historyStore.load()
            _uiState.value = _uiState.value.copy(history = existing)
        }
        scope.launch {
            bleClient.connectionState.collect { state ->
                _uiState.value = _uiState.value.copy(connectionState = state)
            }
        }
        scope.launch {
            bleClient.scanResults.collect { peripherals ->
                _uiState.value = _uiState.value.copy(peripherals = peripherals)
            }
        }
        scope.launch {
            bleClient.events.collect(::handleIncomingEvent)
        }
    }

    suspend fun startScan() {
        scanTimeoutJob?.cancel()
        _uiState.value = _uiState.value.copy(
            scanDebugEntries = emptyList(),
            bleLogEntries = emptyList(),
            errorMessage = null
        )
        bleClient.startScanning()
        scanTimeoutJob = scope.launch {
            delay(12_000)
            bleClient.stopScanning()
            val state = _uiState.value
            _uiState.value = state.copy(
                errorMessage = when {
                    state.peripherals.isNotEmpty() -> "Scan finished"
                    state.scanDebugEntries.isNotEmpty() -> "Scan finished, but no connectable TrafficCam device was identified"
                    else -> "Scan timed out with no BLE advertisements detected"
                }
            )
        }
    }

    suspend fun stopScan() {
        scanTimeoutJob?.cancel()
        bleClient.stopScanning()
    }

    suspend fun pairWith(peripheral: BlePeripheral) {
        scanTimeoutJob?.cancel()
        bleClient.connect(peripheral)
        _uiState.value = _uiState.value.copy(
            pairedDeviceName = peripheral.name ?: peripheral.id,
            peripherals = emptyList(),
        )
    }

    suspend fun requestLatestImage() {
        bleClient.requestLatestImage()
    }

    suspend fun setPaused(paused: Boolean) {
        if (paused) {
            bleClient.pauseMonitoring()
        } else {
            bleClient.resumeMonitoring()
        }
        _uiState.value = _uiState.value.copy(isPaused = paused)
    }

    suspend fun disconnect() {
        scanTimeoutJob?.cancel()
        bleClient.disconnect()
        _uiState.value = _uiState.value.copy(pairedDeviceName = null, peripherals = emptyList())
    }

    suspend fun clearHistory() {
        historySequence = 0
        historyStore.save(emptyList())
        _uiState.value = _uiState.value.copy(history = emptyList(), latestImage = null)
    }

    fun clearError() {
        _uiState.value = _uiState.value.copy(errorMessage = null)
    }

    fun close() {
        scanTimeoutJob?.cancel()
        bleClient.close()
        scope.cancel()
    }

    private suspend fun handleIncomingEvent(event: BleIncomingEvent) {
        when (event) {
            is BleIncomingEvent.ImageReceived -> {
                val historyImage = HistoryImage(
                    mimeType = event.metadata.mimeType,
                    width = event.metadata.width,
                    height = event.metadata.height,
                    bytes = event.bytes,
                )
                attachImageToHistory(
                    event.metadata.reason,
                    event.metadata.receivedAtEpochMillis,
                    historyImage,
                )
                _uiState.value = _uiState.value.copy(latestImage = historyImage)
            }

            is BleIncomingEvent.LogMessage -> {
                _uiState.value = _uiState.value.copy(
                    errorMessage = event.message,
                    bleLogEntries = (listOf(event.message) + _uiState.value.bleLogEntries).take(30),
                )
            }

            is BleIncomingEvent.ScanObservation -> {
                val nextEntries = (listOf(event.message) + _uiState.value.scanDebugEntries)
                    .distinct()
                    .take(20)
                _uiState.value = _uiState.value.copy(scanDebugEntries = nextEntries)
            }

            is BleIncomingEvent.StatusReceived -> {
                val previousState = _uiState.value.latestStatus?.state
                val historyItem = HistoryItem(
                    id = nextHistoryId("status-${event.status.receivedAtEpochMillis}"),
                    kind = HistoryKind.STATUS,
                    title = event.status.state,
                    detail = event.status.summary,
                    createdAtEpochMillis = event.status.capturedAtEpochMillis,
                )
                persistHistory(historyItem)
                _uiState.value = _uiState.value.copy(
                    latestStatus = event.status,
                    isPaused = event.status.state == "Paused",
                )
                if (event.status.state == "Green observed" && previousState != "Green observed") {
                    alertPlayer.playRedOffAlert()
                }
            }
        }
    }

    private suspend fun persistHistory(item: HistoryItem) {
        val nextHistory = listOf(item) + _uiState.value.history
        _uiState.value = _uiState.value.copy(history = nextHistory)
        historyStore.save(nextHistory)
    }

    private suspend fun attachImageToHistory(reason: String, capturedAtEpochMillis: Long, image: HistoryImage) {
        val nextHistory = _uiState.value.history.toMutableList()
        val matchingIndex = nextHistory.indexOfFirst { item ->
            item.image == null &&
                item.kind == HistoryKind.STATUS &&
                item.createdAtEpochMillis in (capturedAtEpochMillis - IMAGE_ASSOCIATION_WINDOW_MS)..(capturedAtEpochMillis + IMAGE_ASSOCIATION_WINDOW_MS)
        }

        if (matchingIndex >= 0) {
            val existing = nextHistory[matchingIndex]
            nextHistory[matchingIndex] = existing.copy(
                detail = if (reason in existing.detail) existing.detail else "${existing.detail}\nImage: $reason",
                image = image,
            )
        } else {
            nextHistory.add(
                0,
                HistoryItem(
                    id = nextHistoryId("image-$capturedAtEpochMillis"),
                    kind = HistoryKind.IMAGE,
                    title = imageTitle(reason),
                    detail = reason,
                    createdAtEpochMillis = capturedAtEpochMillis,
                    image = image,
                ),
            )
        }

        _uiState.value = _uiState.value.copy(history = nextHistory)
        historyStore.save(nextHistory)
    }

    private fun imageTitle(reason: String): String = when (reason) {
        "red_light_detected" -> "Red light image"
        "green_light_detected" -> "Green light image"
        "manual_request" -> "Manual image"
        else -> "Detection image"
    }

    private fun nextHistoryId(prefix: String): String {
        historySequence += 1
        return "$prefix-$historySequence"
    }

    private companion object {
        const val IMAGE_ASSOCIATION_WINDOW_MS = 30_000L
    }
}
