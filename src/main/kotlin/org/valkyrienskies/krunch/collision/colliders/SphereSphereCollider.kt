package org.valkyrienskies.krunch.collision.colliders

import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Pose
import org.valkyrienskies.krunch.collision.CollisionPair
import org.valkyrienskies.krunch.collision.CollisionResult
import org.valkyrienskies.krunch.collision.shapes.SphereShape
import org.valkyrienskies.krunch.computeRelativeVelocity
import kotlin.math.sqrt

object SphereSphereCollider : Collider<SphereShape, SphereShape> {
    override fun computeCollisionBetweenShapes(
        body0Shape: SphereShape,
        body0Transform: Pose,
        body0Velocity: Vector3dc,
        body0AngularVelocity: Vector3dc,
        body1Shape: SphereShape,
        body1Transform: Pose,
        body1Velocity: Vector3dc,
        body1AngularVelocity: Vector3dc,
        dt: Double,
        speculativeThreshold: Double
    ): CollisionResult? {
        val difference = Vector3d(body1Transform.p).sub(body0Transform.p)
        val differenceLengthSq = difference.lengthSquared()
        if (differenceLengthSq > 1e-12) {
            val normal = Vector3d(difference).normalize()
            val body0DeepestPoint =
                body0Transform.invTransform(Vector3d(body0Transform.p).fma(body0Shape.radius, normal))
            val body1DeepestPoint =
                body1Transform.invTransform(Vector3d(body1Transform.p).fma(-body1Shape.radius, normal))

            val relativeVelocity = computeRelativeVelocity(
                normal, body0DeepestPoint, body1DeepestPoint, body0Velocity, body0AngularVelocity, body1Velocity,
                body1AngularVelocity, dt
            )

            if ((body0Shape.radius + body1Shape.radius) > sqrt(
                    differenceLengthSq
                ) + relativeVelocity - speculativeThreshold
            ) {
                return CollisionResult(listOf(CollisionPair(body0DeepestPoint, body1DeepestPoint, normal)))
            }
        }
        return null
    }
}
