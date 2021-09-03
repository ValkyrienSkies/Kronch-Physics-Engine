package org.valkyrienskies.krunch.collision.colliders

import org.joml.Vector3d
import org.valkyrienskies.krunch.Pose
import org.valkyrienskies.krunch.collision.CollisionPair
import org.valkyrienskies.krunch.collision.CollisionResult
import org.valkyrienskies.krunch.collision.shapes.SphereShape

object SphereSphereCollider : Collider<SphereShape, SphereShape> {
    override fun computeCollisionBetweenShapes(
        body0Shape: SphereShape, body0Transform: Pose, body1Shape: SphereShape, body1Transform: Pose
    ): CollisionResult? {
        val difference = Vector3d(body1Transform.p).sub(body0Transform.p)
        val differenceLengthSq = difference.lengthSquared()
        if ((differenceLengthSq > 1e-12) &&
            (differenceLengthSq < (body0Shape.radius + body1Shape.radius) * (body0Shape.radius + body1Shape.radius))
        ) {
            val normal = Vector3d(difference).normalize()
            val body0DeepestPoint =
                body0Transform.invTransform(Vector3d(body0Transform.p).fma(body0Shape.radius, normal))
            val body1DeepestPoint =
                body1Transform.invTransform(Vector3d(body1Transform.p).fma(-body1Shape.radius, normal))
            return CollisionResult(listOf(CollisionPair(body0DeepestPoint, body1DeepestPoint, normal)))
        }
        return null
    }
}
