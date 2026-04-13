package com.greenlightalert.trafficcam.ble

import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.StateFlow

interface BleClient {
    val connectionState: StateFlow<BleConnectionState>
    val scanResults: StateFlow<List<BlePeripheral>>
    val events: Flow<BleIncomingEvent>

    suspend fun startScanning()
    suspend fun stopScanning()
    suspend fun connect(peripheral: BlePeripheral)
    suspend fun disconnect()
    suspend fun requestLatestImage()
    suspend fun pauseMonitoring()
    suspend fun resumeMonitoring()
    fun close() {}
}
