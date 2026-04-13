package com.greenlightalert.trafficcam.ble

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.BluetoothStatusCodes
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.Build
import android.util.Log
import kotlinx.coroutines.channels.BufferOverflow
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.serialization.json.Json
import java.io.ByteArrayOutputStream
import java.nio.charset.StandardCharsets
import java.util.UUID

@SuppressLint("MissingPermission")
class AndroidBleClient(
    context: Context,
) : BleClient {
    private val appContext = context.applicationContext
    private val bluetoothManager = context.getSystemService(BluetoothManager::class.java)
    private val bluetoothAdapter: BluetoothAdapter? = bluetoothManager?.adapter
    private val scanner: BluetoothLeScanner? = bluetoothAdapter?.bluetoothLeScanner
    private val json = Json { ignoreUnknownKeys = true }
    private val _connectionState = MutableStateFlow(BleConnectionState.Idle)
    private val _scanResults = MutableStateFlow(emptyList<BlePeripheral>())
    private val _events = MutableSharedFlow<BleIncomingEvent>(
        replay = 0,
        extraBufferCapacity = 32,
        onBufferOverflow = BufferOverflow.DROP_OLDEST,
    )

    private var gatt: BluetoothGatt? = null
    private var controlCharacteristic: BluetoothGattCharacteristic? = null
    private var currentTransfer: TransferState? = null
    private var scanInProgress = false
    private var pendingNotificationCharacteristics: ArrayDeque<BluetoothGattCharacteristic> = ArrayDeque()
    private var currentTransferChunkCount = 0
    private var awaitingMtu = false
    private var negotiatedMtu = DEFAULT_MTU
    private val pendingImageChunks: ArrayDeque<ByteArray> = ArrayDeque()

    override val connectionState: StateFlow<BleConnectionState> = _connectionState.asStateFlow()
    override val scanResults: StateFlow<List<BlePeripheral>> = _scanResults.asStateFlow()
    override val events: Flow<BleIncomingEvent> = _events.asSharedFlow()

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val deviceName = result.device.name ?: result.scanRecord?.deviceName ?: "Unnamed"
            val peripheral = BlePeripheral(
                id = result.device.address.orEmpty(),
                name = result.device.name ?: result.scanRecord?.deviceName,
                rssi = result.rssi,
            )
            val next = (_scanResults.value + peripheral)
                .associateBy { it.id }
                .values
                .sortedByDescending { it.rssi }
            _scanResults.value = next
            _events.tryEmit(
                BleIncomingEvent.ScanObservation(
                    "$deviceName • ${peripheral.id} • RSSI ${peripheral.rssi}"
                )
            )
        }

        override fun onBatchScanResults(results: MutableList<ScanResult>) {
            _events.tryEmit(
                BleIncomingEvent.LogMessage("BLE batch scan callback with ${results.size} result(s)")
            )
            results.forEach { result ->
                onScanResult(ScanSettings.CALLBACK_TYPE_ALL_MATCHES, result)
            }
        }

        override fun onScanFailed(errorCode: Int) {
            if (errorCode == ScanCallback.SCAN_FAILED_ALREADY_STARTED) {
                _connectionState.value = BleConnectionState.Scanning
                scanInProgress = true
                _events.tryEmit(BleIncomingEvent.LogMessage("BLE scan already running"))
                return
            }
            scanInProgress = false
            _connectionState.value = BleConnectionState.Error
            _events.tryEmit(
                BleIncomingEvent.LogMessage(
                    "BLE scan failed with code $errorCode (${scanFailureLabel(errorCode)})"
                )
            )
        }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            _events.tryEmit(
                BleIncomingEvent.LogMessage(
                    "GATT connection state changed: status=$status, newState=${connectionStateLabel(newState)}"
                )
            )
            if (status != BluetoothGatt.GATT_SUCCESS) {
                _connectionState.value = BleConnectionState.Error
                emitError("GATT error $status")
                this@AndroidBleClient.gatt = null
                gatt.close()
                return
            }

            if (newState == BluetoothProfile.STATE_CONNECTED) {
                this@AndroidBleClient.gatt = gatt
                _connectionState.value = BleConnectionState.Paired
                negotiatedMtu = DEFAULT_MTU
                awaitingMtu = true
                val mtuRequested = gatt.requestMtu(PREFERRED_MTU)
                emitLog("Connected, requesting MTU $PREFERRED_MTU: started=$mtuRequested")
                if (!mtuRequested) {
                    awaitingMtu = false
                    emitLog("MTU request not started, discovering services with default MTU")
                    gatt.discoverServices()
                }
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                _connectionState.value = BleConnectionState.Idle
                controlCharacteristic = null
                currentTransfer = null
                pendingNotificationCharacteristics.clear()
                pendingImageChunks.clear()
                awaitingMtu = false
                negotiatedMtu = DEFAULT_MTU
                this@AndroidBleClient.gatt = null
                emitLog("Disconnected from device")
                gatt.close()
            }
        }

        override fun onMtuChanged(gatt: BluetoothGatt, mtu: Int, status: Int) {
            emitLog("MTU changed: mtu=$mtu, status=$status")
            if (status == BluetoothGatt.GATT_SUCCESS) {
                negotiatedMtu = mtu
            }
            if (!awaitingMtu) {
                return
            }
            awaitingMtu = false
            if (status != BluetoothGatt.GATT_SUCCESS) {
                emitError("MTU request failed with status $status, continuing with default MTU")
            }
            emitLog("Discovering services after MTU negotiation")
            gatt.discoverServices()
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                _connectionState.value = BleConnectionState.Error
                emitError("Service discovery failed: $status")
                return
            }

            val discoveredServices = gatt.services.joinToString { it.uuid.toString() }
            emitLog("Services discovered: $discoveredServices")

            val service = gatt.getService(UUID.fromString(BleProtocol.serviceUuid)) ?: run {
                _connectionState.value = BleConnectionState.Error
                emitError("TrafficCam BLE service not found")
                return
            }

            controlCharacteristic = service.getCharacteristic(UUID.fromString(BleProtocol.controlCharacteristicUuid))
            pendingNotificationCharacteristics.clear()
            service.getCharacteristic(UUID.fromString(BleProtocol.statusCharacteristicUuid))?.let {
                pendingNotificationCharacteristics.add(it)
            }
            service.getCharacteristic(UUID.fromString(BleProtocol.imageMetadataCharacteristicUuid))?.let {
                pendingNotificationCharacteristics.add(it)
            }
            service.getCharacteristic(UUID.fromString(BleProtocol.imageDataCharacteristicUuid))?.let {
                pendingNotificationCharacteristics.add(it)
            }

            if (controlCharacteristic == null) {
                emitError("Control characteristic missing")
            }
            enableNextNotification(gatt)
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, value: ByteArray) {
//            emitLog("characteristic changed: "+ characteristic.uuid.toString())
            when (characteristic.uuid.toString().uppercase()) {
                BleProtocol.statusCharacteristicUuid.uppercase() -> handleStatus(value)
                BleProtocol.imageMetadataCharacteristicUuid.uppercase() -> handleImageMetadata(value)
                BleProtocol.imageDataCharacteristicUuid.uppercase() -> handleImageData(value)
            }
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
            if (Build.VERSION.SDK_INT < Build.VERSION_CODES.TIRAMISU) {
                handleLegacyCharacteristic(characteristic)
            }
        }

        override fun onDescriptorWrite(gatt: BluetoothGatt, descriptor: BluetoothGattDescriptor, status: Int) {
            emitLog(
                "Descriptor write: characteristic=${descriptor.characteristic.uuid}, descriptor=${descriptor.uuid}, status=$status"
            )
            if (status != BluetoothGatt.GATT_SUCCESS) {
                _connectionState.value = BleConnectionState.Error
                emitError("Failed to enable notifications")
                return
            }
            enableNextNotification(gatt)
        }
    }

    override suspend fun startScanning() {
        val adapter = bluetoothAdapter
        if (scanner == null) {
            _events.emit(BleIncomingEvent.LogMessage("Bluetooth LE scanner is unavailable"))
            return
        }
        if (adapter == null || !adapter.isEnabled) {
            _connectionState.value = BleConnectionState.Error
            emitError("Bluetooth is turned off")
            return
        }
        _events.emit(
            BleIncomingEvent.LogMessage(
                "Starting BLE scan: sdk=${Build.VERSION.SDK_INT}, scanner=ok, bluetoothEnabled=${adapter.isEnabled}"
            )
        )
        _scanResults.value = emptyList()
        if (scanInProgress) {
            scanner.stopScan(scanCallback)
            scanInProgress = false
            _events.emit(BleIncomingEvent.LogMessage("Stopped previous BLE scan before restarting"))
        }
        _connectionState.value = BleConnectionState.Scanning
        scanner.startScan(
            emptyList(),
            ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .setReportDelay(0L)
                .build(),
            scanCallback,
        )
        scanInProgress = true
        _events.emit(BleIncomingEvent.LogMessage("BLE scan request submitted"))
    }

    override suspend fun stopScanning() {
        if (scanInProgress) {
            scanner?.stopScan(scanCallback)
            scanInProgress = false
            _events.emit(BleIncomingEvent.LogMessage("BLE scan stopped"))
        }
        if (_connectionState.value == BleConnectionState.Scanning) {
            _connectionState.value = BleConnectionState.Idle
        }
    }

    override suspend fun connect(peripheral: BlePeripheral) {
        stopScanning()
        gatt?.close()
        gatt = null
        controlCharacteristic = null
        currentTransfer = null
        pendingNotificationCharacteristics.clear()
        val device = bluetoothAdapter?.getRemoteDevice(peripheral.id) ?: run {
            emitError("Unable to resolve device ${peripheral.id}")
            return
        }
        _connectionState.value = BleConnectionState.Connecting
        emitLog("Connecting to ${peripheral.name ?: "Unnamed"} (${peripheral.id})")
        gatt = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            device.connectGatt(appContext, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
        } else {
            device.connectGatt(appContext, false, gattCallback)
        }
    }

    override suspend fun disconnect() {
        gatt?.disconnect()
        gatt?.close()
        gatt = null
        controlCharacteristic = null
        currentTransfer = null
        pendingImageChunks.clear()
        _connectionState.value = BleConnectionState.Idle
    }

    override suspend fun requestLatestImage() {
        val gatt = gatt ?: return
        val characteristic = controlCharacteristic ?: return
        characteristic.value = BleProtocol.requestLatestImageCommand.toByteArray(StandardCharsets.UTF_8)
        gatt.writeCharacteristic(characteristic)
    }

    override suspend fun pauseMonitoring() {
        val gatt = gatt ?: return
        val characteristic = controlCharacteristic ?: return
        characteristic.value = BleProtocol.pauseCommand.toByteArray(StandardCharsets.UTF_8)
        gatt.writeCharacteristic(characteristic)
    }

    override suspend fun resumeMonitoring() {
        val gatt = gatt ?: return
        val characteristic = controlCharacteristic ?: return
        characteristic.value = BleProtocol.resumeCommand.toByteArray(StandardCharsets.UTF_8)
        gatt.writeCharacteristic(characteristic)
    }

    override fun close() {
        if (scanInProgress) {
            scanner?.stopScan(scanCallback)
            scanInProgress = false
        }
        gatt?.close()
        gatt = null
        currentTransfer = null
        pendingImageChunks.clear()
    }

    private fun handleLegacyCharacteristic(characteristic: BluetoothGattCharacteristic) {
        val value = characteristic.value ?: return
        when (characteristic.uuid.toString().uppercase()) {
            BleProtocol.statusCharacteristicUuid.uppercase() -> handleStatus(value)
            BleProtocol.imageMetadataCharacteristicUuid.uppercase() -> handleImageMetadata(value)
            BleProtocol.imageDataCharacteristicUuid.uppercase() -> handleImageData(value)
        }
    }

    private fun handleStatus(value: ByteArray) {
        runCatching {
            val payload = json.decodeFromString<BleStatusPayload>(value.toString(StandardCharsets.UTF_8))
            _events.tryEmit(BleIncomingEvent.StatusReceived(payload.toDeviceStatus()))
        }.onFailure {
            emitError("Invalid status payload: ${it.message}")
        }
    }

    private fun handleImageMetadata(value: ByteArray) {
        runCatching {
            val metadata = parseBleImageMetadata(value)
            currentTransfer = TransferState(metadata = metadata)
            currentTransferChunkCount = 0
            emitLog(
                "Image metadata received: id=${metadata.imageId}, reason=${metadata.reason}, bytes=${metadata.totalBytes}, size=${metadata.width}x${metadata.height}, capturedAt=${metadata.capturedAtEpochMillis}, chunkPayload=${metadata.chunkPayloadSize}, totalChunks=${metadata.totalChunks()}, windowChunks=${metadata.windowChunkCount()}, mtu=$negotiatedMtu"
            )
            while (pendingImageChunks.isNotEmpty() && currentTransfer != null) {
                val pendingChunk = pendingImageChunks.removeFirst()
                emitLog("Flushing buffered image chunk: len=${pendingChunk.size}")
                handleImageData(pendingChunk)
            }
        }.onFailure {
            emitError("Invalid image metadata payload: ${it.message}")
        }
    }

    private fun handleImageData(value: ByteArray) {
        val transfer = currentTransfer ?: run {
            if (pendingImageChunks.size < MAX_PENDING_IMAGE_CHUNKS) {
                pendingImageChunks.addLast(value.copyOf())
                emitLog(
                    "Image chunk buffered while waiting for metadata: len=${value.size}, pending=${pendingImageChunks.size}"
                )
            } else {
                emitError("Image chunk received without active metadata transfer: len=${value.size}")
            }
            return
        }
        if (value.size < CHUNK_SEQUENCE_HEADER_SIZE) {
            emitError("Image chunk too small: len=${value.size}")
            currentTransfer = null
            currentTransferChunkCount = 0
            return
        }
        val sequence = readLeUInt16(value, 0)
        if (sequence != transfer.nextChunkSequence) {
            if (transfer.requestedResendFrom != transfer.nextChunkSequence) {
                transfer.requestedResendFrom = transfer.nextChunkSequence
                emitError(
                    "Image chunk sequence mismatch: expected=${transfer.nextChunkSequence}, actual=$sequence, received=${transfer.buffer.size()}/${transfer.metadata.totalBytes}"
                )
                requestResendFromSequence(transfer.nextChunkSequence)
            } else {
                emitLog(
                    "Ignoring out-of-sequence chunk while waiting for resend: expected=${transfer.nextChunkSequence}, actual=$sequence"
                )
            }
            return
        }
        val payload = value.copyOfRange(CHUNK_SEQUENCE_HEADER_SIZE, value.size)
        currentTransferChunkCount += 1
        transfer.nextChunkSequence += 1
        transfer.requestedResendFrom = null
        transfer.buffer.write(payload, 0, payload.size)
        val receivedBytes = transfer.buffer.size()
        if (currentTransferChunkCount <= 3 || receivedBytes >= transfer.metadata.totalBytes || currentTransferChunkCount % 25 == 0) {
            emitLog(
                "Image chunk ${currentTransferChunkCount}: seq=$sequence, len=${payload.size}, received=$receivedBytes/${transfer.metadata.totalBytes}, mtu=$negotiatedMtu"
            )
        }
        if (transfer.buffer.size() >= transfer.metadata.totalBytes) {
            val bytes = transfer.buffer.toByteArray().copyOf(transfer.metadata.totalBytes)
            emitLog("Image rx jpeg edges: ${jpegEdgeSummary(bytes)}")
            if (!looksLikeJpeg(bytes)) {
                emitError(
                    "Image transfer failed JPEG validation: id=${transfer.metadata.imageId}, bytes=${bytes.size}, chunks=$currentTransferChunkCount"
                )
                currentTransfer = null
                currentTransferChunkCount = 0
                pendingImageChunks.clear()
                return
            }
            emitLog(
                "Image transfer complete: id=${transfer.metadata.imageId}, bytes=${bytes.size}, chunks=$currentTransferChunkCount"
            )
            _events.tryEmit(BleIncomingEvent.ImageReceived(transfer.metadata, bytes))
            currentTransfer = null
            currentTransferChunkCount = 0
            pendingImageChunks.clear()
            return
        }
        if (transfer.nextChunkSequence < transfer.metadata.totalChunks() &&
            transfer.nextChunkSequence % transfer.metadata.windowChunkCount() == 0 &&
            transfer.requestedContinueFrom != transfer.nextChunkSequence
        ) {
            transfer.requestedContinueFrom = transfer.nextChunkSequence
            requestContinueFromSequence(transfer.nextChunkSequence)
        }
    }

    private fun enableNotifications(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic?) {
        characteristic ?: run {
            emitError("Notification characteristic missing")
            return
        }
        val localNotificationEnabled = gatt.setCharacteristicNotification(characteristic, true)
        emitLog(
            "setCharacteristicNotification: uuid=${characteristic.uuid}, enabled=$localNotificationEnabled, properties=0x${characteristic.properties.toString(16)}"
        )
        val descriptor = characteristic.getDescriptor(UUID.fromString(CLIENT_CONFIGURATION_DESCRIPTOR))
        if (descriptor == null) {
            emitError("CCCD descriptor missing for ${characteristic.uuid}")
            return
        }
        emitLog("CCCD descriptor found for ${characteristic.uuid}")
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            val status = gatt.writeDescriptor(
                descriptor,
                BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE,
            )
            val started = status == BluetoothStatusCodes.SUCCESS
            emitLog("CCCD write requested: uuid=${characteristic.uuid}, started=$started, status=$status")
            if (!started) {
                emitError("Failed to start CCCD write for ${characteristic.uuid} with status=$status")
            }
        } else {
            descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            val started = gatt.writeDescriptor(descriptor)
            emitLog("CCCD write requested: uuid=${characteristic.uuid}, started=$started")
            if (!started) {
                emitError("Failed to start CCCD write for ${characteristic.uuid}")
            }
        }
    }

    private fun enableNextNotification(gatt: BluetoothGatt) {
        val nextCharacteristic = pendingNotificationCharacteristics.removeFirstOrNull()
        if (nextCharacteristic == null) {
            _connectionState.value = BleConnectionState.Monitoring
            emitLog("Notification setup complete")
            return
        }
        emitLog("Enabling notifications for ${nextCharacteristic.uuid}")
        enableNotifications(gatt, nextCharacteristic)
    }

    private fun emitLog(message: String) {
        Log.d(LOG_TAG, message)
        _events.tryEmit(BleIncomingEvent.LogMessage(message))
    }

    private fun requestResendFromSequence(sequence: Int) {
        val gatt = gatt
        val characteristic = controlCharacteristic
        if (gatt == null || characteristic == null) {
            emitError("Cannot request image resend from sequence $sequence without control characteristic")
            return
        }
        val command = BleProtocol.resendFromSequenceCommand(sequence).toByteArray(StandardCharsets.UTF_8)
        characteristic.value = command
        val started = gatt.writeCharacteristic(characteristic)
        emitLog("Requested image resend from sequence=$sequence started=$started")
        if (!started) {
            emitError("Failed to start image resend request from sequence=$sequence")
        }
    }

    private fun requestContinueFromSequence(sequence: Int) {
        val gatt = gatt
        val characteristic = controlCharacteristic
        if (gatt == null || characteristic == null) {
            emitError("Cannot request image continue from sequence $sequence without control characteristic")
            return
        }
        val command = BleProtocol.continueFromSequenceCommand(sequence).toByteArray(StandardCharsets.UTF_8)
        characteristic.value = command
        val started = gatt.writeCharacteristic(characteristic)
        emitLog("Requested image continue from sequence=$sequence started=$started")
        if (!started) {
            emitError("Failed to start image continue request from sequence=$sequence")
        }
    }

    private fun emitError(message: String) {
        Log.e(LOG_TAG, message)
        _events.tryEmit(BleIncomingEvent.LogMessage(message))
    }

    private data class TransferState(
        val metadata: BleImageMetadata,
        val buffer: ByteArrayOutputStream = ByteArrayOutputStream(metadata.totalBytes),
        var nextChunkSequence: Int = 0,
        var requestedResendFrom: Int? = null,
        var requestedContinueFrom: Int? = null,
    )

    private companion object {
        const val LOG_TAG = "TrafficCamBLE"
        const val CLIENT_CONFIGURATION_DESCRIPTOR = "00002902-0000-1000-8000-00805f9b34fb"
        const val PREFERRED_MTU = 517
        const val DEFAULT_MTU = 23
        const val CHUNK_SEQUENCE_HEADER_SIZE = 2
        const val MAX_PENDING_IMAGE_CHUNKS = 8

        fun scanFailureLabel(errorCode: Int): String = when (errorCode) {
            ScanCallback.SCAN_FAILED_ALREADY_STARTED -> "already started"
            ScanCallback.SCAN_FAILED_APPLICATION_REGISTRATION_FAILED -> "app registration failed"
            ScanCallback.SCAN_FAILED_FEATURE_UNSUPPORTED -> "feature unsupported"
            ScanCallback.SCAN_FAILED_INTERNAL_ERROR -> "internal error"
            ScanCallback.SCAN_FAILED_OUT_OF_HARDWARE_RESOURCES -> "out of hardware resources"
            ScanCallback.SCAN_FAILED_SCANNING_TOO_FREQUENTLY -> "scanning too frequently"
            else -> "unknown"
        }

        fun connectionStateLabel(state: Int): String = when (state) {
            BluetoothProfile.STATE_DISCONNECTED -> "disconnected"
            BluetoothProfile.STATE_CONNECTING -> "connecting"
            BluetoothProfile.STATE_CONNECTED -> "connected"
            BluetoothProfile.STATE_DISCONNECTING -> "disconnecting"
            else -> "unknown($state)"
        }

        fun looksLikeJpeg(bytes: ByteArray): Boolean =
            bytes.size >= 4 &&
                bytes[0] == 0xFF.toByte() &&
                bytes[1] == 0xD8.toByte() &&
                bytes[bytes.lastIndex - 1] == 0xFF.toByte() &&
                bytes[bytes.lastIndex] == 0xD9.toByte()

        fun jpegEdgeSummary(bytes: ByteArray): String {
            if (bytes.isEmpty()) return "empty"
            val head = bytes.take(8).joinToString(" ") { "%02X".format(it.toInt() and 0xFF) }
            val tail = bytes.takeLast(minOf(8, bytes.size)).joinToString(" ") { "%02X".format(it.toInt() and 0xFF) }
            return "head=[$head] tail=[$tail]"
        }

        fun readLeUInt16(bytes: ByteArray, offset: Int): Int =
            (bytes[offset].toInt() and 0xFF) or
                ((bytes[offset + 1].toInt() and 0xFF) shl 8)
    }
}
