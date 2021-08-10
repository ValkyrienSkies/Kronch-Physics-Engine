package org.valkyrienskies.krunch.collision.voxels

interface IVoxelGrid {
    /**
     * Returns true if and only if this [IVoxelGrid] changed.
     */
    fun setVoxelState(x: Int, y: Int, z: Int, setVoxel: Boolean): Boolean

    /**
     * Returns true if and only if the voxel at [x], [y], [z] is set.
     */
    fun getVoxelState(x: Int, y: Int, z: Int): Boolean
}
