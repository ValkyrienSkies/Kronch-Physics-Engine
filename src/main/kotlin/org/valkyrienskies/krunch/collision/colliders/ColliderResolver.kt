package org.valkyrienskies.krunch.collision.colliders

import org.joml.Vector3dc
import org.valkyrienskies.krunch.Posec
import org.valkyrienskies.krunch.collision.CollisionResult
import org.valkyrienskies.krunch.collision.shapes.BoxShape
import org.valkyrienskies.krunch.collision.shapes.CollisionShape
import org.valkyrienskies.krunch.collision.shapes.SphereShape
import org.valkyrienskies.krunch.collision.shapes.TSDFVoxelShape

object ColliderResolver : Collider<CollisionShape, CollisionShape> {
    override fun computeCollisionBetweenShapes(
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
        return when {
            (body0Shape is SphereShape) && (body1Shape is SphereShape) -> {
                SphereSphereCollider.computeCollisionBetweenShapes(
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
            (body0Shape is SphereShape) && (body1Shape is TSDFVoxelShape) -> {
                SphereTSDFVoxelCollider.computeCollisionBetweenShapes(
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
            (body0Shape is SphereShape) && (body1Shape is BoxShape) -> {
                SphereBoxCollider.computeCollisionBetweenShapes(
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
            (body0Shape is TSDFVoxelShape) && (body1Shape is TSDFVoxelShape) -> {
                TSDFVoxelTSDFVoxelCollider.computeCollisionBetweenShapes(
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
            else -> {
                null
            }
        }
    }
}
