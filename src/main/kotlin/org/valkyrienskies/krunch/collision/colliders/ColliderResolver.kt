package org.valkyrienskies.krunch.collision.colliders

import org.valkyrienskies.krunch.Pose
import org.valkyrienskies.krunch.collision.CollisionResultc
import org.valkyrienskies.krunch.collision.shapes.BoxShape
import org.valkyrienskies.krunch.collision.shapes.CollisionShape
import org.valkyrienskies.krunch.collision.shapes.CombinedShape
import org.valkyrienskies.krunch.collision.shapes.SphereShape
import org.valkyrienskies.krunch.collision.shapes.TSDFVoxelShape

object ColliderResolver : Collider<CollisionShape, CollisionShape> {
    override fun computeCollisionBetweenShapes(
        body0Shape: CollisionShape, body0Transform: Pose, body1Shape: CollisionShape, body1Transform: Pose
    ): CollisionResultc? {
        return when {
            (body0Shape is CombinedShape) -> {
                CombinedShapeCollider.computeCollisionBetweenShapes(
                    body0Shape, body0Transform, body1Shape, body1Transform
                )
            }
            (body0Shape is SphereShape) && (body1Shape is SphereShape) -> {
                SphereSphereCollider.computeCollisionBetweenShapes(
                    body0Shape, body0Transform, body1Shape, body1Transform
                )
            }
            (body0Shape is SphereShape) && (body1Shape is TSDFVoxelShape) -> {
                SphereTSDFVoxelCollider.computeCollisionBetweenShapes(
                    body0Shape, body0Transform, body1Shape, body1Transform
                )
            }
            (body0Shape is SphereShape) && (body1Shape is BoxShape) -> {
                SphereBoxCollider.computeCollisionBetweenShapes(
                    body0Shape, body0Transform, body1Shape, body1Transform
                )
            }
            (body0Shape is TSDFVoxelShape) && (body1Shape is TSDFVoxelShape) -> {
                TSDFVoxelTSDFVoxelCollider.computeCollisionBetweenShapes(
                    body0Shape, body0Transform, body1Shape, body1Transform
                )
            }
            else -> {
                null
            }
        }
    }
}
