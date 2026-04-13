package com.greenlightalert.trafficcam.ble

object BleProtocol {
    const val serviceUuid = "7A1C0000-0C4E-4F67-8D80-61A2E5B10000"
    const val statusCharacteristicUuid = "7A1C0001-0C4E-4F67-8D80-61A2E5B10000"
    const val imageMetadataCharacteristicUuid = "7A1C0002-0C4E-4F67-8D80-61A2E5B10000"
    const val imageDataCharacteristicUuid = "7A1C0003-0C4E-4F67-8D80-61A2E5B10000"
    const val controlCharacteristicUuid = "7A1C0004-0C4E-4F67-8D80-61A2E5B10000"

    const val requestLatestImageCommand = """{"command":"request_latest_image"}"""
    const val pauseCommand = """{"command":"pause"}"""
    const val resumeCommand = """{"command":"resume"}"""

    fun resendFromSequenceCommand(sequence: Int): String =
        """{"command":"resend_from","sequence":$sequence}"""

    fun continueFromSequenceCommand(sequence: Int): String =
        """{"command":"continue_from","sequence":$sequence}"""
}
