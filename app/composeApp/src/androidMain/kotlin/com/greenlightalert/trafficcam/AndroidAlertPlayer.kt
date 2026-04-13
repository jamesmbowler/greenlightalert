package com.greenlightalert.trafficcam

import android.media.AudioManager
import android.media.ToneGenerator

class AndroidAlertPlayer : AlertPlayer {
    override fun playRedOffAlert() {
        runCatching {
            val tone = ToneGenerator(AudioManager.STREAM_ALARM, 100)
            tone.startTone(ToneGenerator.TONE_CDMA_ALERT_CALL_GUARD, 1200)
            tone.release()
        }
    }
}
