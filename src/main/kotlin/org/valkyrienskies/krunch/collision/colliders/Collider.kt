package org.valkyrienskies.krunch.collision.colliders

import org.joml.Vector3dc
import org.valkyrienskies.krunch.Posec
import org.valkyrienskies.krunch.collision.CollisionResult
import org.valkyrienskies.krunch.collision.shapes.CollisionShape

/**
 * Computes the collision points between two collision shapes.
 */
interface Collider<in Body0ShapeType : CollisionShape, in Body1ShapeType : CollisionShape> {

    /**
     * This computes speculative contacts using [body0Velocity], [body0AngularVelocity], [body1Velocity], [body1AngularVelocity], [dt], and [speculativeThreshold].
     */
    fun computeCollisionBetweenShapes(
        body0Shape: Body0ShapeType,
        body0Transform: Posec,
        body0Velocity: Vector3dc,
        body0AngularVelocity: Vector3dc,
        body1Shape: Body1ShapeType,
        body1Transform: Posec,
        body1Velocity: Vector3dc,
        body1AngularVelocity: Vector3dc,
        dt: Double,
        speculativeThreshold: Double
    ): CollisionResult?

    /**
     * This should only be used by [ColliderResolver].
     */
    @Suppress("UNCHECKED_CAST")
    fun computeCollisionBetweenShapesGeneric(
        body0Shape: CollisionShape,
        body0Transform: Posec,
        body0Velocity: Vector3dc,
        body0AngularVelocity: Vector3dc,
        body1Shape: CollisionShape,
        body1Transform: Posec,
        body1Velocity: Vector3dc,
        body1AngularVelocity: Vector3dc,
        dt: Double,
        speculativeThreshold: Double
    ): CollisionResult? {
        body0Shape as Body0ShapeType
        body1Shape as Body1ShapeType
        return computeCollisionBetweenShapes(
            body0Shape,
            body0Transform,
            body0Velocity,
            body0AngularVelocity,
            body1Shape,
            body1Transform,
            body1Velocity,
            body1AngularVelocity,
            dt,
            speculativeThreshold
        )
    }
}
