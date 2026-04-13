package com.greenlightalert.trafficcam.ble

import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow

class IosBleClient : BleClient {
    private val _connectionState = MutableStateFlow(BleConnectionState.Idle)
    private val _scanResults = MutableStateFlow(emptyList<BlePeripheral>())
    private val _events = MutableSharedFlow<BleIncomingEvent>(extraBufferCapacity = 16)

    override val connectionState: StateFlow<BleConnectionState> = _connectionState.asStateFlow()
    override val scanResults: StateFlow<List<BlePeripheral>> = _scanResults.asStateFlow()
    override val events: Flow<BleIncomingEvent> = _events.asSharedFlow()

    override suspend fun startScanning() {
        _connectionState.value = BleConnectionState.Scanning
        delay(500)
        _scanResults.value = listOf(
            BlePeripheral(
                id = "SIM-ESP32",
                name = "ESP32 Traffic Cam (demo)",
                rssi = -42,
            )
        )
    }

    override suspend fun stopScanning() {
        _connectionState.value = BleConnectionState.Idle
    }

    override suspend fun connect(peripheral: BlePeripheral) {
        _connectionState.value = BleConnectionState.Connecting
        delay(300)
        _connectionState.value = BleConnectionState.Monitoring
        _events.emit(
            BleIncomingEvent.StatusReceived(
                BleStatusPayload(
                    s = 0,
                    r = 0,
                    g = 0,
                ).toDeviceStatus()
            )
        )
    }

    override suspend fun disconnect() {
        _connectionState.value = BleConnectionState.Idle
    }

    override suspend fun requestLatestImage() {
        _events.emit(BleIncomingEvent.LogMessage("iOS demo client does not request live BLE images yet."))
    }

    override suspend fun pauseMonitoring() {
        _events.emit(BleIncomingEvent.LogMessage("iOS demo client pause requested."))
        _events.emit(
            BleIncomingEvent.StatusReceived(
                BleStatusPayload(s = 6, r = 0, g = 0).toDeviceStatus()
            )
        )
    }

    override suspend fun resumeMonitoring() {
        _events.emit(BleIncomingEvent.LogMessage("iOS demo client resume requested."))
        _events.emit(
            BleIncomingEvent.StatusReceived(
                BleStatusPayload(s = 0, r = 0, g = 0).toDeviceStatus()
            )
        )
    }
}
