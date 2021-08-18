package org.valkyrienskies.krunch.collision.voxels

import org.joml.Vector3d

interface IVoxelLayeredTSDF {

    fun setVoxel(posX: Int, posY: Int, posZ: Int, set: Boolean)

    fun getSignedDistanceAndNormal(
        posX: Double, posY: Double, posZ: Double, collisionNormalOutput: Vector3d,
        signedDistanceOutput: OutputParameterDouble, isQueryValid: OutputParameterBoolean
    )

    // fun getSignedDistance(
    //     posX: Double, posY: Double, posZ: Double,
    //     signedDistanceOutput: OutputParameterDouble, isQueryValid: OutputParameterBoolean
    // )

    data class OutputParameterDouble(var output: Double)
    data class OutputParameterBoolean(var output: Boolean)
}
