package org.valkyrienskies.krunch.collision.shapes

import org.joml.Vector3d
import org.joml.Vector3ic
import org.valkyrienskies.krunch.collision.voxels.BasicLayeredTSDF
import org.valkyrienskies.krunch.collision.voxels.IVoxelLayeredTSDF

class TSDFVoxelShape : CollisionShape {
    // Used in collision to offset the grid of this voxel shape
    var shapeOffset = Vector3d()

    val layeredTSDF: IVoxelLayeredTSDF = BasicLayeredTSDF()

    companion object {
        fun createNewVoxelShape(initialVoxels: Iterable<Vector3ic>): TSDFVoxelShape {
            val newVoxelShape = TSDFVoxelShape()
            initialVoxels.forEach {
                newVoxelShape.layeredTSDF.setVoxel(it.x(), it.y(), it.z(), true)
            }
            return newVoxelShape
        }
    }
}
