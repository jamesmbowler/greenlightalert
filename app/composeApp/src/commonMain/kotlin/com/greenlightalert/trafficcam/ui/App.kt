package com.greenlightalert.trafficcam.ui

import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeDrawing
import androidx.compose.foundation.layout.windowInsetsPadding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.itemsIndexed
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.mutableStateMapOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.layout.ContentScale
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.ui.text.style.TextDecoration
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import com.greenlightalert.trafficcam.AppUiState
import com.greenlightalert.trafficcam.DeviceStatus
import com.greenlightalert.trafficcam.HistoryImage
import com.greenlightalert.trafficcam.HistoryItem
import com.greenlightalert.trafficcam.HistoryKind
import com.greenlightalert.trafficcam.ManageRuntimeLifecycle
import com.greenlightalert.trafficcam.TrafficCamRuntime
import com.greenlightalert.trafficcam.asTimestampLabel
import com.greenlightalert.trafficcam.decodeBleImage
import com.greenlightalert.trafficcam.ble.BleConnectionState
import kotlinx.coroutines.launch
import kotlin.time.Clock
import kotlin.time.ExperimentalTime

@Composable
fun TrafficCamApp(runtime: TrafficCamRuntime) {
    ManageRuntimeLifecycle(runtime)
    MaterialTheme {
        val scope = rememberCoroutineScope()
        val uiState by runtime.presenter.uiState.collectAsState()
        var currentScreen by remember { mutableStateOf(Screen.Home) }

        Scaffold { padding ->
            Surface(
                modifier = Modifier
                    .fillMaxSize()
                    .windowInsetsPadding(androidx.compose.foundation.layout.WindowInsets.safeDrawing)
                    .background(
                        brush = Brush.verticalGradient(
                            listOf(
                                MaterialTheme.colorScheme.surfaceContainerHighest,
                                MaterialTheme.colorScheme.surface,
                            )
                        )
                    )
                    .padding(padding),
            ) {
                when (currentScreen) {
                    Screen.Home -> {
                        LazyColumn(
                            modifier = Modifier.fillMaxSize().padding(16.dp),
                            verticalArrangement = Arrangement.spacedBy(16.dp),
                        ) {
                            item {
                                HeroCard(
                                    uiState = uiState,
                                    onOpenLogs = { currentScreen = Screen.Logs },
                                    onOpenHistory = { currentScreen = Screen.History },
                                )
                            }
                            item {
                                PairingCard(
                                    uiState = uiState,
                                    onScan = { scope.launch { runtime.presenter.startScan() } },
                                    onStop = { scope.launch { runtime.presenter.stopScan() } },
                                    onPair = { peripheral -> scope.launch { runtime.presenter.pairWith(peripheral) } },
                                    onDisconnect = { scope.launch { runtime.presenter.disconnect() } },
                                    onRequestLatestImage = { scope.launch { runtime.presenter.requestLatestImage() } },
                                    onTogglePause = { paused -> scope.launch { runtime.presenter.setPaused(paused) } },
                                )
                            }
                            item {
                                LatestImageCard(uiState.latestImage)
                            }
                        }
                    }

                    Screen.Logs -> {
                        LogsScreen(
                            bleLogEntries = uiState.bleLogEntries,
                            scanDebugEntries = uiState.scanDebugEntries,
                            onBack = { currentScreen = Screen.Home },
                        )
                    }

                    Screen.History -> {
                        HistoryScreen(
                            latestStatus = uiState.latestStatus,
                            history = uiState.history,
                            onClearHistory = { scope.launch { runtime.presenter.clearHistory() } },
                            onBack = { currentScreen = Screen.Home },
                        )
                    }
                }
            }
        }
    }
}

@OptIn(ExperimentalTime::class)
@Composable
private fun HeroCard(uiState: AppUiState, onOpenLogs: () -> Unit, onOpenHistory: () -> Unit) {
    val liveStatus = uiState.latestStatus.takeIf { status ->
        status?.let {
            Clock.System.now().toEpochMilliseconds() - (status.receivedAtEpochMillis) <= 5_000
        } ?: false
    }
    Card {
        Column(modifier = Modifier.fillMaxWidth().padding(20.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Traffic Cam Monitor", style = MaterialTheme.typography.headlineMedium)
            CurrentStatusBlock(liveStatus)
            Text("Connection: ${uiState.connectionState.name}", style = MaterialTheme.typography.labelLarge)
            uiState.errorMessage?.let {
                Text(
                    text = "BLE: $it",
                    style = MaterialTheme.typography.bodyMedium,
                    color = MaterialTheme.colorScheme.error,
                )
            }
            Text(
                text = "View logs",
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.primary,
                textDecoration = TextDecoration.Underline,
                modifier = Modifier.clickable(onClick = onOpenLogs),
            )
            Text(
                text = "View history",
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.primary,
                textDecoration = TextDecoration.Underline,
                modifier = Modifier.clickable(onClick = onOpenHistory),
            )
        }
    }
}

@Composable
private fun LogsScreen(
    bleLogEntries: List<String>,
    scanDebugEntries: List<String>,
    onBack: () -> Unit,
) {
    LazyColumn(
        modifier = Modifier.fillMaxSize().padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp),
    ) {
        item {
            Card {
                Column(
                    modifier = Modifier.fillMaxWidth().padding(20.dp),
                    verticalArrangement = Arrangement.spacedBy(8.dp),
                ) {
                    Text("Logs", style = MaterialTheme.typography.headlineMedium)
                    Button(onClick = onBack) { Text("Back") }
                }
            }
        }
        item {
            Card {
                Column(
                    modifier = Modifier.fillMaxWidth().padding(20.dp),
                    verticalArrangement = Arrangement.spacedBy(8.dp),
                ) {
                    Text("BLE Session Log", style = MaterialTheme.typography.titleLarge)
                    if (bleLogEntries.isEmpty()) {
                        Text("No BLE session logs yet.", style = MaterialTheme.typography.bodySmall)
                    } else {
                        bleLogEntries.forEach { entry ->
                            Text(entry, style = MaterialTheme.typography.bodySmall)
                        }
                    }
                }
            }
        }
        item {
            Card {
                Column(
                    modifier = Modifier.fillMaxWidth().padding(20.dp),
                    verticalArrangement = Arrangement.spacedBy(8.dp),
                ) {
                    Text("BLE Scan Debug", style = MaterialTheme.typography.titleLarge)
                    if (scanDebugEntries.isEmpty()) {
                        Text("No scan debug entries yet.", style = MaterialTheme.typography.bodySmall)
                    } else {
                        scanDebugEntries.forEach { entry ->
                            Text(entry, style = MaterialTheme.typography.bodySmall)
                        }
                    }
                }
            }
        }
    }
}

@OptIn(ExperimentalTime::class)
@Composable
private fun HistoryScreen(
    latestStatus: DeviceStatus?,
    history: List<HistoryItem>,
    onClearHistory: () -> Unit,
    onBack: () -> Unit,
) {
    val liveStatus = latestStatus.takeIf { status ->
        status?.let {
            Clock.System.now().toEpochMilliseconds() - status.receivedAtEpochMillis <= 5_000
        } ?: false
    }
    val sortedHistory = history.sortedByDescending { it.createdAtEpochMillis }
    val groupedHistory = remember(sortedHistory) { buildHistoryGroups(sortedHistory) }
    val expandedGroups = remember { mutableStateMapOf<String, Boolean>() }

    LazyColumn(
        modifier = Modifier.fillMaxSize().padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp),
    ) {
        item {
            Card {
                Column(
                    modifier = Modifier.fillMaxWidth().padding(20.dp),
                    verticalArrangement = Arrangement.spacedBy(8.dp),
                ) {
                    Text("History", style = MaterialTheme.typography.headlineMedium)
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        Button(onClick = onBack) { Text("Back") }
                        Button(onClick = onClearHistory) { Text("Clear history") }
                    }
                }
            }
        }
        item {
            Card {
                Column(
                    modifier = Modifier.fillMaxWidth().padding(20.dp),
                    verticalArrangement = Arrangement.spacedBy(8.dp),
                ) {
                    Text("Current status", style = MaterialTheme.typography.titleLarge)
                    CurrentStatusBlock(liveStatus)
                }
            }
        }
        itemsIndexed(groupedHistory, key = { index, group -> "${group.key}-$index" }) { _, group ->
            when (group) {
                is HistoryGroup.Single -> HistoryCard(group.item)
                is HistoryGroup.StatusCluster -> {
                    val expanded = expandedGroups[group.key] == true
                    StatusClusterCard(
                        group = group,
                        expanded = expanded,
                        onToggleExpanded = { expandedGroups[group.key] = !expanded },
                    )
                }
            }
        }
    }
}

@Composable
private fun CurrentStatusBlock(status: DeviceStatus?) {
    if (status == null) {
        Text(
            text = "No recent status in the last 5 seconds.",
            style = MaterialTheme.typography.bodyLarge,
        )
        return
    }

    Text(
        text = "Current status: ${status.state}",
        style = MaterialTheme.typography.titleMedium,
        fontWeight = FontWeight.SemiBold,
    )
    Text(status.summary, style = MaterialTheme.typography.bodyLarge)
    Text("Received: ${status.receivedAtEpochMillis.asTimestampLabel()}", style = MaterialTheme.typography.labelMedium)
    Text("Device time: ${status.capturedAtEpochMillis} ms", style = MaterialTheme.typography.labelSmall)
}

@Composable
private fun PairingCard(
    uiState: AppUiState,
    onScan: () -> Unit,
    onStop: () -> Unit,
    onPair: (com.greenlightalert.trafficcam.ble.BlePeripheral) -> Unit,
    onDisconnect: () -> Unit,
    onRequestLatestImage: () -> Unit,
    onTogglePause: (Boolean) -> Unit,
) {
    val matchingPeripherals = uiState.peripherals.filter { peripheral ->
        peripheral.name
            ?.lowercase()
            ?.replace(" ", "")
            ?.contains("greenlightalert") == true
    }
    val isConnected = uiState.connectionState == BleConnectionState.Paired ||
        uiState.connectionState == BleConnectionState.Monitoring ||
        uiState.connectionState == BleConnectionState.Connecting
    Card {
        Column(modifier = Modifier.fillMaxWidth().padding(20.dp), verticalArrangement = Arrangement.spacedBy(12.dp)) {
            Text("Pairing", style = MaterialTheme.typography.titleLarge)
            Text(
                text = uiState.pairedDeviceName?.let { "Paired with $it" } ?: "No device paired yet.",
                style = MaterialTheme.typography.bodyMedium,
            )
            Text(
                text = if (uiState.isPaused) "Monitoring is paused." else "Monitoring is active.",
                style = MaterialTheme.typography.bodyMedium,
                color = if (uiState.isPaused) MaterialTheme.colorScheme.error else MaterialTheme.colorScheme.primary,
            )
            PairingActions(
                onScan = onScan,
                onStop = onStop,
                onDisconnect = onDisconnect,
                onRequestLatestImage = onRequestLatestImage,
                isPaused = uiState.isPaused,
                onTogglePause = onTogglePause,
            )
            if (!isConnected) {
                Text(
                    text = when {
                        matchingPeripherals.isNotEmpty() -> "${matchingPeripherals.size} GreenLightAlert device(s) found"
                        else -> "No GreenLightAlert devices found yet."
                    },
                    style = MaterialTheme.typography.labelLarge,
                )
            }
            if (!isConnected && matchingPeripherals.isNotEmpty()) {
                Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                    matchingPeripherals.forEach { peripheral ->
                        Card(onClick = { onPair(peripheral) }) {
                            Row(
                                modifier = Modifier.fillMaxWidth().padding(12.dp),
                                horizontalArrangement = Arrangement.SpaceBetween,
                                verticalAlignment = Alignment.CenterVertically,
                            ) {
                                Column {
                                    Text(peripheral.name ?: "Unnamed camera", fontWeight = FontWeight.Medium)
                                    Text(peripheral.id, style = MaterialTheme.typography.bodySmall)
                                }
                                Text("RSSI ${peripheral.rssi}")
                            }
                        }
                    }
                }
            }
        }
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun PairingActions(
    onScan: () -> Unit,
    onStop: () -> Unit,
    onDisconnect: () -> Unit,
    onRequestLatestImage: () -> Unit,
    isPaused: Boolean,
    onTogglePause: (Boolean) -> Unit,
) {
    FlowRow(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
        maxItemsInEachRow = 2,
    ) {
        Button(onClick = onScan) { Text("Scan") }
        Button(onClick = onStop) { Text("Stop") }
        Button(onClick = onDisconnect) { Text("Disconnect") }
        Button(onClick = onRequestLatestImage) { Text("Pull image") }
        Button(onClick = { onTogglePause(!isPaused) }) {
            Text(if (isPaused) "Resume" else "Pause")
        }
    }
}

@Composable
private fun LatestImageCard(image: HistoryImage?) {
    Card {
        Column(modifier = Modifier.fillMaxWidth().padding(20.dp), verticalArrangement = Arrangement.spacedBy(12.dp)) {
            Text("Latest detection image", style = MaterialTheme.typography.titleLarge)
            if (image == null) {
                Text("No image received yet.")
            } else {
                HistoryImageView(image = image, modifier = Modifier.fillMaxWidth().aspectRatio(1f))
            }
        }
    }
}

@Composable
private fun HistoryCard(item: HistoryItem) {
    Card {
        Column(modifier = Modifier.fillMaxWidth().padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(item.title, fontWeight = FontWeight.SemiBold)
            Text(item.detail, style = MaterialTheme.typography.bodyMedium)
            Text(item.createdAtEpochMillis.asTimestampLabel(), style = MaterialTheme.typography.labelSmall)
            item.image?.let {
                Spacer(Modifier.height(4.dp))
                HistoryImageView(it, modifier = Modifier.fillMaxWidth().aspectRatio(1f))
            }
        }
    }
}

@Composable
private fun StatusClusterCard(
    group: HistoryGroup.StatusCluster,
    expanded: Boolean,
    onToggleExpanded: () -> Unit,
) {
    val newest = group.items.first()
    val newestImage = group.items.firstNotNullOfOrNull { it.image }
    Card {
        Column(modifier = Modifier.fillMaxWidth().padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(newest.title, fontWeight = FontWeight.SemiBold)
            Text(newest.detail, style = MaterialTheme.typography.bodyMedium)
            Text(
                "${group.items.size} updates from ${newest.createdAtEpochMillis.asTimestampLabel()} to ${group.items.last().createdAtEpochMillis.asTimestampLabel()}",
                style = MaterialTheme.typography.labelSmall,
            )
            newestImage?.let {
                Spacer(Modifier.height(4.dp))
                HistoryImageView(it, modifier = Modifier.fillMaxWidth().aspectRatio(1f))
            }
            if (group.items.size > 1) {
                Text(
                    text = if (expanded) "Collapse" else "Expand",
                    style = MaterialTheme.typography.bodyMedium,
                    color = MaterialTheme.colorScheme.primary,
                    textDecoration = TextDecoration.Underline,
                    modifier = Modifier.clickable(onClick = onToggleExpanded),
                )
            }
            if (expanded) {
                group.items.forEach { item ->
                    Column(verticalArrangement = Arrangement.spacedBy(4.dp)) {
                        Text(item.createdAtEpochMillis.asTimestampLabel(), style = MaterialTheme.typography.labelSmall)
                        if (item !== newest) {
                            Text(item.detail, style = MaterialTheme.typography.bodySmall)
                        }
                        item.image?.takeIf { it !== newestImage }?.let {
                            HistoryImageView(it, modifier = Modifier.fillMaxWidth().aspectRatio(1f))
                        }
                    }
                }
            }
        }
    }
}

@Composable
private fun HistoryImageView(image: HistoryImage, modifier: Modifier = Modifier) {
    val bitmap = decodeBleImage(image.bytes)
    if (bitmap == null) {
        Box(modifier = modifier.clip(RoundedCornerShape(18.dp)).background(MaterialTheme.colorScheme.surfaceVariant)) {
            Text(
                text = "Unable to decode ${image.mimeType}",
                modifier = Modifier.align(Alignment.Center),
            )
        }
    } else {
        Image(
            bitmap = bitmap,
            contentDescription = "Detection image",
            modifier = modifier.clip(RoundedCornerShape(18.dp)),
            contentScale = ContentScale.Fit,
        )
    }
}

private sealed interface HistoryGroup {
    val key: String

    data class Single(val item: HistoryItem) : HistoryGroup {
        override val key: String = item.id
    }

    data class StatusCluster(val items: List<HistoryItem>) : HistoryGroup {
        override val key: String = items.first().id
    }
}

private fun buildHistoryGroups(items: List<HistoryItem>): List<HistoryGroup> {
    val groups = mutableListOf<HistoryGroup>()
    var index = 0
    while (index < items.size) {
        val item = items[index]
        if (item.kind != HistoryKind.STATUS) {
            groups += HistoryGroup.Single(item)
            index += 1
            continue
        }

        val clusteredItems = mutableListOf(item)
        var nextIndex = index + 1
        while (nextIndex < items.size) {
            val next = items[nextIndex]
            if (next.kind != HistoryKind.STATUS || next.title != item.title || next.detail != item.detail) {
                break
            }
            clusteredItems += next
            nextIndex += 1
        }

        groups += if (clusteredItems.size == 1) {
            HistoryGroup.Single(item)
        } else {
            HistoryGroup.StatusCluster(clusteredItems)
        }
        index = nextIndex
    }
    return groups
}

private enum class Screen {
    Home,
    Logs,
    History,
}
