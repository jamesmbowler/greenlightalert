package com.greenlightalert.trafficcam.history

import android.content.Context
import com.greenlightalert.trafficcam.HistoryItem
import com.greenlightalert.trafficcam.HistoryImage
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import kotlinx.serialization.builtins.ListSerializer
import kotlinx.serialization.json.Json
import java.io.File

class AndroidHistoryStore(
    context: Context,
) : HistoryStore {
    private val file = File(context.filesDir, "trafficcam_history.json")
    private val imagesDir = File(context.filesDir, "trafficcam_images")
    private val json = Json { ignoreUnknownKeys = true; prettyPrint = true }

    override suspend fun load(): List<HistoryItem> = withContext(Dispatchers.IO) {
        if (!file.exists()) return@withContext emptyList()
        runCatching {
            json.decodeFromString(ListSerializer(HistoryItem.serializer()), file.readText()).map { item ->
                item.copy(image = item.image?.let(::loadImageBytes))
            }
        }.getOrDefault(emptyList())
    }

    override suspend fun save(items: List<HistoryItem>) = withContext(Dispatchers.IO) {
        if (items.isEmpty()) {
            file.writeText(json.encodeToString(ListSerializer(HistoryItem.serializer()), emptyList()))
            imagesDir.deleteRecursively()
            return@withContext
        }
        imagesDir.mkdirs()
        val persistedItems = items.map { item ->
            item.copy(image = item.image?.let { persistImage(item.id, it) })
        }
        pruneUnusedImages(persistedItems)
        file.writeText(json.encodeToString(ListSerializer(HistoryItem.serializer()), persistedItems))
    }

    private fun persistImage(itemId: String, image: HistoryImage): HistoryImage {
        val fileName = image.storedFileName ?: "$itemId.jpg"
        val imageFile = File(imagesDir, fileName)
        if (image.bytes.isNotEmpty()) {
            imageFile.writeBytes(image.bytes)
        }
        return image.copy(storedFileName = fileName, bytes = byteArrayOf())
    }

    private fun loadImageBytes(image: HistoryImage): HistoryImage {
        val fileName = image.storedFileName ?: return image
        val imageFile = File(imagesDir, fileName)
        val bytes = if (imageFile.exists()) imageFile.readBytes() else byteArrayOf()
        return image.copy(bytes = bytes)
    }

    private fun pruneUnusedImages(items: List<HistoryItem>) {
        val usedNames = items.mapNotNull { it.image?.storedFileName }.toSet()
        if (!imagesDir.exists()) return
        imagesDir.listFiles()?.forEach { imageFile ->
            if (imageFile.name !in usedNames) {
                imageFile.delete()
            }
        }
    }
}
