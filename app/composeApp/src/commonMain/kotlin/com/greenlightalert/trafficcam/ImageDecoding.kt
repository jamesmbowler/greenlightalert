package com.greenlightalert.trafficcam

import androidx.compose.ui.graphics.ImageBitmap

expect fun decodeBleImage(bytes: ByteArray): ImageBitmap?
