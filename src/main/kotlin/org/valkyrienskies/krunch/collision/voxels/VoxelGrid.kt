package org.valkyrienskies.krunch.collision.voxels

import java.util.BitSet

class VoxelGrid(
    private val minX: Int, private val minY: Int, private val minZ: Int, private val maxX: Int, private val maxY: Int,
    private val maxZ: Int
) {
    private val size: Int
    private val voxelDataBitSet: BitSet

    init {
        // Sanity check
        assert(maxX >= minX) { "MaxX must be >= than MinX" }
        assert(maxY >= minY) { "MaxY must be >= than MinY" }
        assert(maxZ >= minZ) { "MaxZ must be >= than MinZ" }

        size = (maxX - minX + 1) * (maxY - minY + 1) * (maxZ - minZ + 1)
        voxelDataBitSet = BitSet(size)
    }

    internal fun setVoxelState(x: Int, y: Int, z: Int, setVoxel: Boolean) {
        assertPosWithinGrid(x, y, z)
        val bitIndex = posToBitIndex(x, y, z)
        voxelDataBitSet.set(bitIndex, setVoxel)
    }

    internal fun getVoxelState(x: Int, y: Int, z: Int): Boolean {
        assertPosWithinGrid(x, y, z)
        val bitIndex = posToBitIndex(x, y, z)
        return voxelDataBitSet.get(bitIndex)
    }

    private fun posToBitIndex(x: Int, y: Int, z: Int): Int {
        return (x - minX) * (maxY - minY + 1) * (maxZ - minZ + 1) + (y - minY) * (maxZ - minZ + 1) + (z - minZ)
    }

    private fun assertPosWithinGrid(x: Int, y: Int, z: Int) {
        if (x < minX || x > maxX) throw IllegalArgumentException(
            "x with value $x must be within minX $minX and maxX $maxX!"
        )
        if (y < minY || y > maxY) throw IllegalArgumentException(
            "y with value $y must be within minY $minY and maxY $maxY!"
        )
        if (z < minZ || z > maxZ) throw IllegalArgumentException(
            "z with value $z must be within minZ $minZ and maxZ $maxZ!"
        )
    }
}
