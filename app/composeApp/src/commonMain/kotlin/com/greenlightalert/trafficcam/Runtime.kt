package com.greenlightalert.trafficcam

import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect

class TrafficCamRuntime(
    val presenter: TrafficCamPresenter,
)

@Composable
expect fun rememberTrafficCamRuntime(): TrafficCamRuntime

@Composable
fun ManageRuntimeLifecycle(runtime: TrafficCamRuntime) {
    DisposableEffect(runtime) {
        onDispose { runtime.presenter.close() }
    }
}
