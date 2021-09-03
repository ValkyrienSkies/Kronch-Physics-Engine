package org.valkyrienskies.krunch.collision

import org.joml.Vector3dc

/**
 * Immutable view of [CollisionPair].
 *
 * Note that only [positionInFirstBody], [positionInSecondBody], and [originalCollisionNormal] are immutable.
 *
 * The other fields are updated during sub-steps and iterations.
 */
interface CollisionPairc {
    val positionInFirstBody: Vector3dc
    val positionInSecondBody: Vector3dc
    val originalCollisionNormal: Vector3dc
    val skipThisSubStep: Boolean
    val normalThisSubStep: Vector3dc
    val usedThisSubStep: Boolean
    val normalLambdaThisSubStep: Double
    val tangentialLambdaThisSubStep: Double
}
