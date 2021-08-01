package org.valkyrienskies.krunch.collision.colliders

import org.valkyrienskies.krunch.Pose
import org.valkyrienskies.krunch.collision.CollisionResultc
import org.valkyrienskies.krunch.collision.shapes.CollisionShape

/**
 * Computes the collision points between two collision shapes.
 */
interface Collider<in Body0ShapeType : CollisionShape, in Body1ShapeType : CollisionShape> {
    fun computeCollisionBetweenShapes(
        body0Shape: Body0ShapeType, body0Transform: Pose, body1Shape: Body1ShapeType,
        body1Transform: Pose
    ): CollisionResultc?
}
