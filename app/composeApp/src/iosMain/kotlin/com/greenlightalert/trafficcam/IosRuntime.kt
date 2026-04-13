package com.greenlightalert.trafficcam

import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import com.greenlightalert.trafficcam.ble.IosBleClient
import com.greenlightalert.trafficcam.history.IosHistoryStore

@Composable
actual fun rememberTrafficCamRuntime(): TrafficCamRuntime = remember {
    TrafficCamRuntime(
        presenter = TrafficCamPresenter(
            bleClient = IosBleClient(),
            historyStore = IosHistoryStore(),
            alertPlayer = IosAlertPlayer(),
        ),
    )
}
