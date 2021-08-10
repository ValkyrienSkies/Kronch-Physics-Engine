package org.valkyrienskies.krunch.collision.voxels

import it.unimi.dsi.fastutil.longs.Long2IntMap
import it.unimi.dsi.fastutil.longs.Long2IntOpenHashMap
import it.unimi.dsi.fastutil.longs.LongArrayList

class LongBackedPositionSet : PositionSet {

    private val positionsList = LongArrayList()
    private val positionToListIndexMap: Long2IntMap = Long2IntOpenHashMap()

    override fun addPosition(x: Int, y: Int, z: Int): Boolean {
        val posAsLong = posToLong(x, y, z)
        if (positionToListIndexMap.containsKey(posAsLong)) return false // Position already present, no changes needed

        // Add the position
        positionToListIndexMap[posAsLong] = positionsList.size
        positionsList.add(posAsLong)
        return true
    }

    override fun removePosition(x: Int, y: Int, z: Int): Boolean {
        val posAsLong = posToLong(x, y, z)
        if (!positionToListIndexMap.containsKey(
                posAsLong
            )
        ) return false // Position already not present, no changes needed

        // Remove the position
        val positionIndex = positionToListIndexMap.remove(posAsLong)

        if (positionIndex == positionsList.size - 1) {
            // Remove the last element of the list
            positionsList.removeLong(positionsList.size - 1)
        } else {
            // Swap positionIndex with the back of the list
            val backOfListPosition = positionsList.getLong(positionsList.size - 1)
            positionsList.set(positionIndex, backOfListPosition)
            positionToListIndexMap[backOfListPosition] = positionIndex
            // Remove the last element of the list
            positionsList.removeLong(positionsList.size - 1)
        }

        return true
    }

    override fun containsPosition(x: Int, y: Int, z: Int): Boolean {
        val posAsLong = posToLong(x, y, z)
        return positionToListIndexMap.containsKey(posAsLong)
    }

    override fun iterateOverAllPositions(function: (x: Int, y: Int, z: Int) -> Unit) =
        iterateOverAllPositionsInline(function)

    /**
     * An inline version of [iterateOverAllPositions] because interfaces can't have inline functions.
     */
    internal inline fun iterateOverAllPositionsInline(function: (x: Int, y: Int, z: Int) -> Unit) {
        for (i in 0 until positionsList.size) {
            val positionLong = positionsList.getLong(i)
            longToPos(positionLong, function)
        }
    }

    private fun posToLong(x: Int, y: Int, z: Int): Long {
        val xBits = x.toLong() and 0xFFFFF.toLong() // Only include 20 bits from [x]
        val yBits = y.toLong() and 0xFFFFF.toLong() // Only include 20 bits from [y]
        val zBits = z.toLong() and 0xFFFFF.toLong() // Only include 20 bits from [z]
        return (xBits shl 40) or (yBits shl 20) or zBits
    }

    private inline fun longToPos(posLong: Long, function: (x: Int, y: Int, z: Int) -> Unit) {
        val x: Int
        val y: Int
        val z: Int

        var xBits = (posLong shr 40) and 0xFFFFF.toLong()
        if ((xBits and 0x80000.toLong()) != 0L)
            xBits = xBits or 0xFFFFF.toLong().inv() // If x is negative, then sign extend
        x = xBits.toInt()

        var yBits = (posLong shr 20) and 0xFFFFF.toLong()
        if ((yBits and 0x80000.toLong()) != 0L)
            yBits = yBits or 0xFFFFF.toLong().inv() // If y is negative, then sign extend
        y = yBits.toInt()

        var zBits = (posLong) and 0xFFFFF.toLong()
        if ((zBits and 0x80000.toLong()) != 0L)
            zBits = zBits or 0xFFFFF.toLong().inv() // If z is negative, then sign extend
        z = zBits.toInt()

        function(x, y, z)
    }
}
