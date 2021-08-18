package org.valkyrienskies.krunch.collision.voxels

import org.joml.Vector3d
import org.valkyrienskies.krunch.collision.voxels.IVoxelLayeredTSDF.OutputParameterBoolean
import org.valkyrienskies.krunch.collision.voxels.IVoxelLayeredTSDF.OutputParameterDouble
import org.valkyrienskies.krunch.datastructures.Int3HashSet
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt
import kotlin.math.sqrt

/**
 * A basic slow implementation of [IVoxelLayeredTSDF]
 */
class BasicLayeredTSDF : IVoxelLayeredTSDF {

    private val voxelStorage = Int3HashSet()

    override fun setVoxel(posX: Int, posY: Int, posZ: Int, set: Boolean) {
        if (set)
            voxelStorage.add(posX, posY, posZ)
        else
            voxelStorage.remove(posX, posY, posZ)
    }

    private fun getVoxel(posX: Int, posY: Int, posZ: Int): Boolean = voxelStorage.contains(posX, posY, posZ)

    override fun getSignedDistanceAndNormal(
        posX: Double, posY: Double, posZ: Double, collisionNormalOutput: Vector3d,
        signedDistanceOutput: OutputParameterDouble, isQueryValid: OutputParameterBoolean
    ) {
        val centerVoxelGridX = posX.roundToInt()
        val centerVoxelGridY = posY.roundToInt()
        val centerVoxelGridZ = posZ.roundToInt()

        val isPointInsideSolidVoxel: Boolean = getVoxel(centerVoxelGridX, centerVoxelGridY, centerVoxelGridZ)

        isQueryValid.output = false

        for (voxelX in centerVoxelGridX - 1..centerVoxelGridX + 1) {
            for (voxelY in centerVoxelGridY - 1..centerVoxelGridY + 1) {
                for (voxelZ in centerVoxelGridZ - 1..centerVoxelGridZ + 1) {
                    val voxelSolid = getVoxel(voxelX, voxelY, voxelZ)
                    if (voxelSolid xor isPointInsideSolidVoxel) {
                        computeDistanceAndNormal(
                            posX, posY, posZ, voxelX, voxelY, voxelZ
                        ) { skip: Boolean, absoluteDistance: Double, normalX: Double, normalY: Double, normalZ: Double ->
                            if (!skip) {
                                if (!isQueryValid.output) {
                                    signedDistanceOutput.output = absoluteDistance
                                    collisionNormalOutput.set(normalX, normalY, normalZ)
                                    isQueryValid.output = true
                                } else {
                                    if (absoluteDistance < signedDistanceOutput.output) {
                                        signedDistanceOutput.output = absoluteDistance
                                        collisionNormalOutput.set(normalX, normalY, normalZ)
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if (!isQueryValid.output) {
            if (!isPointInsideSolidVoxel) {
                signedDistanceOutput.output = -signedDistanceOutput.output
            }
        }
    }

    private inline fun computeDistanceAndNormal(
        posX: Double, posY: Double, posZ: Double, voxelX: Int, voxelY: Int, voxelZ: Int,
        function: (skip: Boolean, absoluteDistance: Double, normalX: Double, normalY: Double, normalZ: Double) -> Unit
    ) {

        val closestPointX = min(max(posX, voxelX - .5), voxelX - .5)
        val closestPointY = min(max(posY, voxelY - .5), voxelY - .5)
        val closestPointZ = min(max(posZ, voxelZ - .5), voxelZ - .5)

        val absoluteDistance = sqrt(
            (closestPointX - posX) * (closestPointX - posX) + (closestPointY - posY) * (closestPointY - posY) + (closestPointZ - posZ) * (closestPointZ - posZ)
        )

        if (absoluteDistance < 1e-6) {
            function(true, 0.0, 0.0, 0.0, 0.0)
        } else {
            val normalX = (closestPointX - posX) / absoluteDistance
            val normalY = (closestPointY - posY) / absoluteDistance
            val normalZ = (closestPointZ - posZ) / absoluteDistance

            function(false, absoluteDistance, normalX, normalY, normalZ)
        }
    }

    fun forEachVoxel(function: (posX: Int, posY: Int, posZ: Int) -> Unit) = voxelStorage.forEach(function)
}
