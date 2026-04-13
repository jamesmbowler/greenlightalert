package com.greenlightalert.trafficcam

import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.ui.platform.LocalContext
import com.greenlightalert.trafficcam.ble.AndroidBleClient
import com.greenlightalert.trafficcam.history.AndroidHistoryStore

@Composable
actual fun rememberTrafficCamRuntime(): TrafficCamRuntime {
    val context = LocalContext.current.applicationContext
    return remember {
        TrafficCamRuntime(
            presenter = TrafficCamPresenter(
                bleClient = AndroidBleClient(context),
                historyStore = AndroidHistoryStore(context),
                alertPlayer = AndroidAlertPlayer(),
            ),
        )
    }
}
