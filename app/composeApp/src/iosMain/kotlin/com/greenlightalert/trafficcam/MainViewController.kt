package com.greenlightalert.trafficcam

import androidx.compose.ui.window.ComposeUIViewController
import com.greenlightalert.trafficcam.ui.TrafficCamApp
import platform.UIKit.UIViewController

fun MainViewController(): UIViewController = ComposeUIViewController {
    TrafficCamApp(runtime = rememberTrafficCamRuntime())
}
