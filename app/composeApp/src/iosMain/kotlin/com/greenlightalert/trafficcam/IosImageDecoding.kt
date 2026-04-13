package com.greenlightalert.trafficcam

import androidx.compose.ui.graphics.ImageBitmap
import org.jetbrains.skia.Image
import org.jetbrains.skia.toComposeImageBitmap

actual fun decodeBleImage(bytes: ByteArray): ImageBitmap? = runCatching {
    Image.makeFromEncoded(bytes).toComposeImageBitmap()
}.getOrNull()
