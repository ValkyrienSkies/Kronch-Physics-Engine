package org.valkyrienskies.krunch.collision.shapes

import org.joml.Vector3d
import org.joml.Vector3ic
import org.valkyrienskies.krunch.collision.voxels.BasicLayeredTSDF
import org.valkyrienskies.krunch.collision.voxels.IVoxelLayeredTSDF.OutputParameterBoolean
import org.valkyrienskies.krunch.collision.voxels.IVoxelLayeredTSDF.OutputParameterDouble

class NewVoxelShape : CollisionShape {
    // Used in collision to offset the grid of this voxel shape
    var shapeOffset = Vector3d()

    private val layeredTSDF = BasicLayeredTSDF()

    fun forEachVoxel(function: (posX: Int, posY: Int, posZ: Int) -> Unit) = layeredTSDF.forEachVoxel(function)

    fun getSignedDistanceAndNormal(
        posX: Double, posY: Double, posZ: Double, collisionNormalOutput: Vector3d,
        signedDistanceOutput: OutputParameterDouble, isQueryValid: OutputParameterBoolean
    ) = layeredTSDF.getSignedDistanceAndNormal(
        posX, posY, posZ, collisionNormalOutput,
        signedDistanceOutput, isQueryValid
    )

    companion object {
        fun createNewVoxelShape(initialVoxels: Iterable<Vector3ic>): NewVoxelShape {
            val newVoxelShape = NewVoxelShape()
            initialVoxels.forEach {
                newVoxelShape.layeredTSDF.setVoxel(it.x(), it.y(), it.z(), true)
            }
            return newVoxelShape
        }
    }
}
