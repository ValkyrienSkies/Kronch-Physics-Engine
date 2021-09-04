package org.valkyrienskies.krunch.collision

import org.joml.Vector3d
import org.joml.Vector3dc

/**
 * A [CollisionPair] describes how to resolve a collision using two points.
 *
 * Both of these points must be pushed apart along [normalThisSubStep] to resolve the collision.
 */
class CollisionPair(
    override val positionInFirstBody: Vector3dc,
    override val positionInSecondBody: Vector3dc,
    override val originalCollisionNormal: Vector3dc
) : CollisionPairc {
    /**
     * Set [skipThisSubStep] to true when its impossible to determine a normal.
     * (For example, when contact points are exactly on top of each other, the displacement is 0, so no normal exists)
     */
    override var skipThisSubStep: Boolean = false
    override var normalThisSubStep: Vector3d = Vector3d()
    override var usedThisSubStep: Boolean = false
    override var normalLambdaThisSubStep: Double = 0.0
    override var tangentialLambdaThisSubStep: Double = 0.0
}
