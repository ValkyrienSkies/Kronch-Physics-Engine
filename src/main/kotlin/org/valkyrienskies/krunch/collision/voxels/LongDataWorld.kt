package org.valkyrienskies.krunch.collision.voxels

import it.unimi.dsi.fastutil.ints.Int2ObjectMap
import it.unimi.dsi.fastutil.ints.Int2ObjectOpenHashMap
import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap

/**
 * This class stores a long for each block position.
 */
internal class LongDataWorld(private val chunkSizeBits: Int = 4) {

    private val dataMap = Long2ObjectOpenHashMap<Int2ObjectMap<LongDataChunk>>()

    fun setDataAt(posX: Int, posY: Int, posZ: Int, data: Long) {
        val chunkX = posX shr chunkSizeBits
        val chunkY = posY shr chunkSizeBits
        val chunkZ = posZ shr chunkSizeBits

        val relativeX = posX - (chunkX shl chunkSizeBits)
        val relativeY = posY - (chunkY shl chunkSizeBits)
        val relativeZ = posZ - (chunkZ shl chunkSizeBits)

        val horizontalKey = convertTwoIntToLong(posX, posZ)

        if (!dataMap.containsKey(horizontalKey)) dataMap[horizontalKey] = Int2ObjectOpenHashMap()
        val yMap = dataMap[horizontalKey]

        if (!yMap.containsKey(chunkY)) yMap[chunkY] =
            LongDataChunk(1 shl chunkSizeBits, 1 shl chunkSizeBits, 1 shl chunkSizeBits)

        yMap[chunkY].setDataAt(relativeX, relativeY, relativeZ, data)
    }

    fun getDataAt(posX: Int, posY: Int, posZ: Int): Long {
        val chunkX = posX shr chunkSizeBits
        val chunkY = posY shr chunkSizeBits
        val chunkZ = posZ shr chunkSizeBits

        val relativeX = posX - (chunkX shl chunkSizeBits)
        val relativeY = posY - (chunkY shl chunkSizeBits)
        val relativeZ = posZ - (chunkZ shl chunkSizeBits)

        val horizontalKey = convertTwoIntToLong(posX, posZ)

        if (!dataMap.containsKey(horizontalKey)) return EMPTY_DATA
        val yMap = dataMap[horizontalKey]

        if (!yMap.containsKey(chunkY)) return EMPTY_DATA

        return yMap[chunkY].getDataAt(relativeX, relativeY, relativeZ)
    }

    /**
     * Returns the long of a concatenated with b
     */
    private fun convertTwoIntToLong(a: Int, b: Int): Long {
        return (a.toLong() shl 32) or (b.toLong() and 0xFFFFFFFF)
    }

    companion object {
        const val EMPTY_DATA: Long = 0
    }
}
