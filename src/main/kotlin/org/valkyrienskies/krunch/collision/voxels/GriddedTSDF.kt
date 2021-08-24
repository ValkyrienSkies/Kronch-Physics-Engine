package org.valkyrienskies.krunch.collision.voxels

import org.joml.Vector3d

class GriddedTSDF : IVoxelLayeredTSDF {
    private val voxelWorld = LongDataWorld()

    override fun setVoxel(posX: Int, posY: Int, posZ: Int, set: Boolean) {
        TODO("Not yet implemented")
    }

    override fun getVoxel(posX: Int, posY: Int, posZ: Int): Boolean {
        TODO("Not yet implemented")
    }

    override fun forEachVoxel(function: (posX: Int, posY: Int, posZ: Int) -> Unit) {
        TODO("Not yet implemented")
    }

    override fun getClosestPoint(posX: Double, posY: Double, posZ: Double, closestPointOutput: Vector3d): Boolean {
        TODO("Not yet implemented")
    }

    private inline fun iterate3by3(function: (xOffset: Int, yOffset: Int, zOffset: Int) -> Unit) {
        for (xOffset in -1..1) {
            for (yOffset in -1..1) {
                for (zOffset in -1..1) {
                    function(xOffset, yOffset, zOffset)
                }
            }
        }
    }

    companion object {
        // private fun distanceToIndex()
    }
}
