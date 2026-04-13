package com.greenlightalert.trafficcam

import platform.AudioToolbox.AudioServicesPlaySystemSound

class IosAlertPlayer : AlertPlayer {
    override fun playRedOffAlert() {
        AudioServicesPlaySystemSound(1005u)
    }
}

