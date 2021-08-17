package org.valkyrienskies.krunch.collision.voxels

internal class LongDataChunk(
    val sizeX: Int = 16, val sizeY: Int = 16, val sizeZ: Int = 16
) {
    private val dataArray = LongArray(sizeX * sizeY * sizeZ)

    private fun positionToIndex(posX: Int, posY: Int, posZ: Int): Int {
        return posX + posY * sizeX + posZ * sizeX * sizeY
    }

    fun setDataAt(posX: Int, posY: Int, posZ: Int, data: Long) {
        dataArray[positionToIndex(posX, posY, posZ)] = data
    }

    fun getDataAt(posX: Int, posY: Int, posZ: Int): Long {
        return dataArray[positionToIndex(posX, posY, posZ)]
    }
}
