package org.valkyrienskies.krunch.collision.voxels

import it.unimi.dsi.fastutil.ints.Int2IntRBTreeMap
import it.unimi.dsi.fastutil.ints.Int2IntSortedMap
import org.joml.primitives.AABBd

/**
 * This generates AABBs for voxel shapes by keeping track of the number of voxels in all the xyz cross sections of the
 * voxel shape.
 */
class BasicVoxelShapeAABBGenerator : VoxelShapeAABBGenerator {

    private val xCrossSectionVoxelCount: Int2IntSortedMap = Int2IntRBTreeMap()
    private val yCrossSectionVoxelCount: Int2IntSortedMap = Int2IntRBTreeMap()
    private val zCrossSectionVoxelCount: Int2IntSortedMap = Int2IntRBTreeMap()

    override fun setVoxel(x: Int, y: Int, z: Int) {
        xCrossSectionVoxelCount[x] = xCrossSectionVoxelCount.getOrDefault(x, 0) + 1
        yCrossSectionVoxelCount[y] = yCrossSectionVoxelCount.getOrDefault(y, 0) + 1
        zCrossSectionVoxelCount[z] = zCrossSectionVoxelCount.getOrDefault(z, 0) + 1
    }

    override fun unsetVoxel(x: Int, y: Int, z: Int) {
        xCrossSectionVoxelCount[x] = xCrossSectionVoxelCount.get(x) - 1
        yCrossSectionVoxelCount[y] = yCrossSectionVoxelCount.get(y) - 1
        zCrossSectionVoxelCount[z] = zCrossSectionVoxelCount.get(z) - 1
    }

    override fun getAABB(dest: AABBd): AABBd {
        dest.minX = xCrossSectionVoxelCount.firstIntKey().toDouble() - .5
        dest.minY = yCrossSectionVoxelCount.firstIntKey().toDouble() - .5
        dest.minZ = zCrossSectionVoxelCount.firstIntKey().toDouble() - .5
        dest.maxX = xCrossSectionVoxelCount.lastIntKey().toDouble() + .5
        dest.maxY = yCrossSectionVoxelCount.lastIntKey().toDouble() + .5
        dest.maxZ = zCrossSectionVoxelCount.lastIntKey().toDouble() + .5
        return dest
    }
}
