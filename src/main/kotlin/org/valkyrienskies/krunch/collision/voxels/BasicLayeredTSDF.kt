package org.valkyrienskies.krunch.collision.voxels

import org.joml.Vector3d
import org.joml.primitives.AABBd
import org.valkyrienskies.krunch.datastructures.Int3HashSet
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

/**
 * A basic slow implementation of [IVoxelLayeredTSDF]
 */
class BasicLayeredTSDF : IVoxelLayeredTSDF {

    private val voxelStorage = Int3HashSet()
    private val aabbGenerator: VoxelShapeAABBGenerator = BasicVoxelShapeAABBGenerator()

    override fun setVoxel(posX: Int, posY: Int, posZ: Int, set: Boolean) {
        if (set) {
            val changed = voxelStorage.add(posX, posY, posZ)
            if (changed) aabbGenerator.setVoxel(posX, posY, posZ)
        } else {
            val changed = voxelStorage.remove(posX, posY, posZ)
            if (changed) aabbGenerator.unsetVoxel(posX, posY, posZ)
        }
    }

    override fun getVoxel(posX: Int, posY: Int, posZ: Int): Boolean = voxelStorage.contains(posX, posY, posZ)

    override fun getClosestPoint(
        posX: Double, posY: Double, posZ: Double, closestPointOutput: Vector3d
    ): Boolean {
        val centerVoxelGridX = posX.roundToInt()
        val centerVoxelGridY = posY.roundToInt()
        val centerVoxelGridZ = posZ.roundToInt()

        val isPointInsideSolidVoxel: Boolean = getVoxel(centerVoxelGridX, centerVoxelGridY, centerVoxelGridZ)

        var validOutput = false

        var closestDistanceSq = Double.MAX_VALUE

        for (voxelX in centerVoxelGridX - 1..centerVoxelGridX + 1) {
            for (voxelY in centerVoxelGridY - 1..centerVoxelGridY + 1) {
                for (voxelZ in centerVoxelGridZ - 1..centerVoxelGridZ + 1) {
                    val voxelSolid = getVoxel(voxelX, voxelY, voxelZ)
                    if (voxelSolid xor isPointInsideSolidVoxel) {
                        computeClosestPoint(
                            posX, posY, posZ, voxelX, voxelY, voxelZ
                        ) { absoluteDistanceSq: Double, closestX: Double, closestY: Double, closestZ: Double ->
                            if (!validOutput) {
                                closestPointOutput.set(closestX, closestY, closestZ)
                                closestDistanceSq = absoluteDistanceSq
                                validOutput = true
                            } else {
                                if (absoluteDistanceSq < closestDistanceSq) {
                                    closestPointOutput.set(closestX, closestY, closestZ)
                                    closestDistanceSq = absoluteDistanceSq
                                }
                            }
                        }
                    }
                }
            }
        }

        return validOutput
    }

    private inline fun computeClosestPoint(
        posX: Double, posY: Double, posZ: Double, voxelX: Int, voxelY: Int, voxelZ: Int,
        function: (absoluteDistance: Double, closestX: Double, closestY: Double, closestZ: Double) -> Unit
    ) {
        val closestPointX = min(max(posX, voxelX - .5), voxelX + .5)
        val closestPointY = min(max(posY, voxelY - .5), voxelY + .5)
        val closestPointZ = min(max(posZ, voxelZ - .5), voxelZ + .5)

        val absoluteDistanceSq =
            (closestPointX - posX) * (closestPointX - posX) +
                (closestPointY - posY) * (closestPointY - posY) +
                (closestPointZ - posZ) * (closestPointZ - posZ)

        function(absoluteDistanceSq, closestPointX, closestPointY, closestPointZ)
    }

    override fun forEachVoxel(function: (posX: Int, posY: Int, posZ: Int) -> Unit) = voxelStorage.forEach(function)

    override fun getAABB(dest: AABBd): AABBd = aabbGenerator.getAABB(dest)
}
