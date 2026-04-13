package com.greenlightalert.trafficcam.history

import com.greenlightalert.trafficcam.HistoryItem

interface HistoryStore {
    suspend fun load(): List<HistoryItem>
    suspend fun save(items: List<HistoryItem>)
}
