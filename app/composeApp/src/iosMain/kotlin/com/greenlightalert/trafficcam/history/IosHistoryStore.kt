package com.greenlightalert.trafficcam.history

import com.greenlightalert.trafficcam.HistoryItem
import com.greenlightalert.trafficcam.HistoryImage
import kotlinx.cinterop.addressOf
import kotlinx.cinterop.ExperimentalForeignApi
import kotlinx.cinterop.usePinned
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import kotlinx.serialization.builtins.ListSerializer
import kotlinx.serialization.json.Json
import platform.Foundation.NSData
import platform.Foundation.NSDocumentDirectory
import platform.Foundation.NSFileManager
import platform.Foundation.NSString
import platform.Foundation.NSUTF8StringEncoding
import platform.Foundation.NSSearchPathForDirectoriesInDomains
import platform.Foundation.NSUserDomainMask
import platform.Foundation.create
import platform.Foundation.dataWithContentsOfFile
import platform.Foundation.stringWithContentsOfFile
import platform.Foundation.writeToFile

class IosHistoryStore : HistoryStore {
    private val json = Json { ignoreUnknownKeys = true; prettyPrint = true }

    override suspend fun load(): List<HistoryItem> = withContext(Dispatchers.Default) {
        val path = historyFilePath()
        val content = NSString.stringWithContentsOfFile(path, NSUTF8StringEncoding, null) ?: return@withContext emptyList()
        runCatching {
            json.decodeFromString(ListSerializer(HistoryItem.serializer()), content.toString()).map { item ->
                item.copy(image = item.image?.let(::loadImageBytes))
            }
        }.getOrDefault(emptyList())
    }

    override suspend fun save(items: List<HistoryItem>) = withContext(Dispatchers.Default) {
        val path = historyFilePath()
        if (items.isEmpty()) {
            NSString.create(string = json.encodeToString(ListSerializer(HistoryItem.serializer()), emptyList()))
                .writeToFile(path, atomically = true, encoding = NSUTF8StringEncoding, error = null)
            NSFileManager.defaultManager.removeItemAtPath(imagesDirectoryPath(), null)
            return@withContext
        }
        ensureImagesDirectory()
        val persistedItems = items.map { item ->
            item.copy(image = item.image?.let { persistImage(item.id, it) })
        }
        pruneUnusedImages(persistedItems)
        val content = json.encodeToString(ListSerializer(HistoryItem.serializer()), persistedItems)
        NSString.create(string = content).writeToFile(path, atomically = true, encoding = NSUTF8StringEncoding, error = null)
    }

    private fun persistImage(itemId: String, image: HistoryImage): HistoryImage {
        val fileName = image.storedFileName ?: "$itemId.jpg"
        val path = "${imagesDirectoryPath()}/$fileName"
        if (image.bytes.isNotEmpty()) {
            NSData.create(bytes = image.bytes.refTo(0), length = image.bytes.size.toULong())
                .writeToFile(path, atomically = true)
        }
        return image.copy(storedFileName = fileName, bytes = byteArrayOf())
    }

    private fun loadImageBytes(image: HistoryImage): HistoryImage {
        val fileName = image.storedFileName ?: return image
        val path = "${imagesDirectoryPath()}/$fileName"
        val data = NSData.dataWithContentsOfFile(path) ?: return image
        val bytes = ByteArray(data.length.toInt())
        bytes.usePinned { pinned ->
            platform.posix.memcpy(pinned.addressOf(0), data.bytes, data.length)
        }
        return image.copy(bytes = bytes)
    }

    private fun pruneUnusedImages(items: List<HistoryItem>) {
        val usedNames = items.mapNotNull { it.image?.storedFileName }.toSet()
        val fileManager = NSFileManager.defaultManager
        val imagesPath = imagesDirectoryPath()
        val files = fileManager.contentsOfDirectoryAtPath(imagesPath, null, null) as? List<*> ?: return
        files.filterIsInstance<String>().forEach { name ->
            if (name !in usedNames) {
                fileManager.removeItemAtPath("$imagesPath/$name", null)
            }
        }
    }

    private fun ensureImagesDirectory() {
        NSFileManager.defaultManager.createDirectoryAtPath(imagesDirectoryPath(), true, null, null)
    }

    @OptIn(ExperimentalForeignApi::class)
    private fun historyFilePath(): String {
        val directory = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, true)
            .firstOrNull() as? String
            ?: ""
        return "$directory/trafficcam_history.json"
    }

    @OptIn(ExperimentalForeignApi::class)
    private fun imagesDirectoryPath(): String {
        val directory = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, true)
            .firstOrNull() as? String
            ?: ""
        return "$directory/trafficcam_images"
    }
}
