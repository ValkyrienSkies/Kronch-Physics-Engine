package org.valkyrienskies.krunch.collision.shapes

import org.joml.Vector3d
import org.joml.Vector3ic
import org.joml.primitives.AABBd
import org.valkyrienskies.krunch.collision.voxels.BasicLayeredTSDF
import org.valkyrienskies.krunch.collision.voxels.IVoxelLayeredTSDF

class TSDFVoxelShape : CollisionShape {
    /**
     * The voxel position to local position is defined as the following:
     *
     * localPos = (voxelPos + voxelOffset) * scalingFactor
     */
    var voxelOffset = Vector3d()
    var scalingFactor = 1.0

    val layeredTSDF: IVoxelLayeredTSDF = BasicLayeredTSDF()

    override val sortIndex: Int = 0

    override fun getAABB(dest: AABBd): AABBd {
        layeredTSDF.getAABB(dest)
        dest.translate(voxelOffset)
        dest.minX *= scalingFactor
        dest.minY *= scalingFactor
        dest.minZ *= scalingFactor
        dest.maxX *= scalingFactor
        dest.maxY *= scalingFactor
        dest.maxZ *= scalingFactor
        return dest
    }

    fun getVoxelCount(): Long = layeredTSDF.getVoxelCount()

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
