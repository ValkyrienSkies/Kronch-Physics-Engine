package org.valkyrienskies.krunch.collision.colliders

import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Pose
import org.valkyrienskies.krunch.collision.CollisionPair
import org.valkyrienskies.krunch.collision.CollisionResult
import org.valkyrienskies.krunch.collision.CollisionResultc
import org.valkyrienskies.krunch.collision.shapes.BoxShape
import org.valkyrienskies.krunch.collision.shapes.SphereShape
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

object SphereBoxCollider : Collider<SphereShape, BoxShape> {
    override fun computeCollisionBetweenShapes(
        body0Shape: SphereShape, body0Transform: Pose, body1Shape: BoxShape, body1Transform: Pose
    ): CollisionResultc? {
        val spherePosRelativeToBox: Vector3dc = body1Transform.invTransform(Vector3d(body0Transform.p))

        val closestPointRelativeToBoxInBody1Coordinates = Vector3d(
            min(body1Shape.xRadius, max(-body1Shape.xRadius, spherePosRelativeToBox.x())),
            min(body1Shape.yRadius, max(-body1Shape.yRadius, spherePosRelativeToBox.y())),
            min(body1Shape.zRadius, max(-body1Shape.zRadius, spherePosRelativeToBox.z()))
        )

        val isSphereCenterWithinBox =
            (abs(spherePosRelativeToBox.x()) < body1Shape.xRadius)
                && (abs(spherePosRelativeToBox.y()) < body1Shape.yRadius)
                && (abs(spherePosRelativeToBox.z()) < body1Shape.zRadius)

        if ((closestPointRelativeToBoxInBody1Coordinates.lengthSquared() < body0Shape.radius * body0Shape.radius)) {
            val difference =
                body1Transform.transform(Vector3d(closestPointRelativeToBoxInBody1Coordinates)).sub(body0Transform.p)
            if (difference.lengthSquared() > 1e-12) {
                val normal = Vector3d(difference).normalize()
                val deepestSpherePoint = Vector3d(body0Transform.p)
                if (!isSphereCenterWithinBox) {
                    deepestSpherePoint.fma(body0Shape.radius, normal)
                } else {
                    deepestSpherePoint.fma(-body0Shape.radius, normal)
                }
                return CollisionResult(
                    listOf(
                        CollisionPair(
                            body0Transform.invTransform(Vector3d(deepestSpherePoint)),
                            closestPointRelativeToBoxInBody1Coordinates,
                            normal
                        )
                    )
                )
            }
        }

        return null
    }
}
