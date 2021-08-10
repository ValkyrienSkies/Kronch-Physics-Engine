package org.valkyrienskies.krunch.collision.voxels

class VoxelGridWithPositionSet(private val baseVoxelGrid: VoxelGrid) : IVoxelGrid {

    private val positionSet: LongBackedPositionSet = LongBackedPositionSet()

    override fun setVoxelState(x: Int, y: Int, z: Int, setVoxel: Boolean): Boolean {
        if (setVoxel) {
            positionSet.addPosition(x, y, z)
        } else {
            positionSet.removePosition(x, y, z)
        }
        return baseVoxelGrid.setVoxelState(x, y, z, setVoxel)
    }

    override fun getVoxelState(x: Int, y: Int, z: Int): Boolean = baseVoxelGrid.getVoxelState(x, y, z)
}
