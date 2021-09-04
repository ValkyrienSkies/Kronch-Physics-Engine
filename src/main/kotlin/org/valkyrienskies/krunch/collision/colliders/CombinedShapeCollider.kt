package org.valkyrienskies.krunch.collision.colliders

import org.joml.Vector3dc
import org.valkyrienskies.krunch.Posec
import org.valkyrienskies.krunch.collision.CollisionResult
import org.valkyrienskies.krunch.collision.shapes.CollisionShape
import org.valkyrienskies.krunch.collision.shapes.CombinedShape

object CombinedShapeCollider : Collider<CombinedShape, CollisionShape> {
    override fun computeCollisionBetweenShapes(
        body0Shape: CombinedShape,
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
        return null
        /*
        var minDepth = Double.MAX_VALUE
        var minCollisionPair: CollisionPair? = null
        for (pair in body0Shape.collisionShapes) {
            val shape = pair.first
            val pose = pair.second
            val collisionResult =
                ColliderResolver.computeCollisionBetweenShapes(shape, pose, body1Shape, body1Transform)

            collisionResult?.collisionPoints?.forEach { it ->
                val body0CollisionPointGlobal: Vector3dc = body0Transform.transform(
                    pose.transform(
                        Vector3d(it.positionInFirstBody)
                    )
                )
                val body1CollisionPointGlobal: Vector3dc = body1Transform.transform(
                    Vector3d(it.positionInSecondBody)
                )
                val difference = Vector3d(body1CollisionPointGlobal).sub(body0CollisionPointGlobal)

                val depth = it.originalCollisionNormal.dot(difference)

                if (depth < minDepth) {
                    minDepth = depth
                    minCollisionPair = CollisionPair(
                        body0Transform.invTransform(Vector3d(body0CollisionPointGlobal)), it.positionInSecondBody,
                        it.originalCollisionNormal
                    )
                }
            }
        }
        return if (minCollisionPair == null) {
            null
        } else {
            CollisionResult(listOf(minCollisionPair!!))
        }

         */
    }
}
