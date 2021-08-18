package org.valkyrienskies.krunch.collision.voxels

import org.joml.Vector3d
import org.valkyrienskies.krunch.collision.voxels.IVoxelLayeredTSDF.OutputParameterBoolean
import org.valkyrienskies.krunch.collision.voxels.IVoxelLayeredTSDF.OutputParameterDouble

class GriddedTSDF : IVoxelLayeredTSDF {
    private val voxelWorld = LongDataWorld()

    override fun setVoxel(posX: Int, posY: Int, posZ: Int, set: Boolean) {
        if (set) {

        }
        TODO("Not yet implemented")
    }

    override fun getSignedDistanceAndNormal(
        posX: Double, posY: Double, posZ: Double, collisionNormalOutput: Vector3d,
        signedDistanceOutput: OutputParameterDouble, isQueryValid: OutputParameterBoolean
    ) {
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
