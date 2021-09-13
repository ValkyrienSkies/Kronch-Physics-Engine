package org.valkyrienskies.krunch.collision.voxels

import org.joml.Vector3d
import org.joml.Vector3dc
import org.joml.primitives.AABBd
import kotlin.math.roundToInt

interface IVoxelLayeredTSDF {

    fun setVoxel(posX: Int, posY: Int, posZ: Int, set: Boolean)

    fun getVoxel(posX: Int, posY: Int, posZ: Int): Boolean

    fun getVoxel(posX: Double, posY: Double, posZ: Double): Boolean =
        getVoxel(posX.roundToInt(), posY.roundToInt(), posZ.roundToInt())

    fun getVoxel(pos: Vector3dc): Boolean = getVoxel(pos.x(), pos.y(), pos.z())

    fun forEachVoxel(function: (posX: Int, posY: Int, posZ: Int) -> Unit)

    fun getAABB(dest: AABBd = AABBd()): AABBd

    /**
     * Stores the closest surface point to ([posX], [posY], [posZ]) in [closestPointOutput].
     *
     * Returns true if the closest surface point was found, false if it wasn't.
     *
     * In other words, if the position is too far away from the surface, then we give up and return false.
     */
    fun getClosestPoint(posX: Double, posY: Double, posZ: Double, closestPointOutput: Vector3d): Boolean

    fun getVoxelCount(): Long
}
