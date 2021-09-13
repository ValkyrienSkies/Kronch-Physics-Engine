package org.valkyrienskies.krunch.collision.voxels

import org.joml.primitives.AABBd

/**
 * Generates the AABB for voxel shapes.
 */
interface VoxelShapeAABBGenerator {
    fun setVoxel(x: Int, y: Int, z: Int)
    fun unsetVoxel(x: Int, y: Int, z: Int)
    fun getAABB(dest: AABBd = AABBd()): AABBd
}
