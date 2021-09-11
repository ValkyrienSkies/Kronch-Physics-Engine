package org.valkyrienskies.krunch.collision

import org.joml.Vector3dc

/**
 * A [CollisionPair] describes how to resolve a collision using two points.
 *
 * Both of these points must be pushed apart along [originalCollisionNormal] to resolve the collision.
 */
class CollisionPair(
    override val positionInFirstBody: Vector3dc,
    override val positionInSecondBody: Vector3dc,
    override val originalCollisionNormal: Vector3dc
) : CollisionPairc
