package org.valkyrienskies.krunch.collision.colliders

import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Pose
import org.valkyrienskies.krunch.collision.CollisionPair
import org.valkyrienskies.krunch.collision.CollisionResult
import org.valkyrienskies.krunch.collision.shapes.SphereShape
import org.valkyrienskies.krunch.collision.shapes.TSDFVoxelShape

object SphereTSDFVoxelCollider : Collider<SphereShape, TSDFVoxelShape> {
    override fun computeCollisionBetweenShapes(
        body0Shape: SphereShape, body0Transform: Pose, body1Shape: TSDFVoxelShape, body1Transform: Pose
    ): CollisionResult? {
        val spherePosInBody1Coordinates: Vector3dc = body1Transform.invTransform(Vector3d(body0Transform.p))

        val closestSurfacePointOutput = Vector3d()

        val isQueryValid = body1Shape.layeredTSDF.getClosestPoint(
            spherePosInBody1Coordinates.x(), spherePosInBody1Coordinates.y(), spherePosInBody1Coordinates.z(),
            closestSurfacePointOutput
        )

        if (isQueryValid) {
            val isSphereCenterInVoxel = body1Shape.layeredTSDF.getVoxel(spherePosInBody1Coordinates)

            val deepestSpherePoint: Vector3dc = if (!isSphereCenterInVoxel) {
                val normal = Vector3d(closestSurfacePointOutput).sub(spherePosInBody1Coordinates).normalize()
                Vector3d(spherePosInBody1Coordinates).fma(body0Shape.radius, normal)
            } else {
                val normal = Vector3d(spherePosInBody1Coordinates).sub(closestSurfacePointOutput).normalize()
                Vector3d(spherePosInBody1Coordinates).fma(body0Shape.radius, normal)
            }

            if (body1Shape.layeredTSDF.getVoxel(deepestSpherePoint)) {
                val normal = Vector3d(closestSurfacePointOutput).sub(deepestSpherePoint)
                body1Transform.rotate(normal)
                normal.mul(-1.0)
                normal.normalize()

                val deepestPointInBody0Coordinates =
                    body0Transform.invTransform(body1Transform.transform(Vector3d(deepestSpherePoint)))

                return CollisionResult(
                    listOf(CollisionPair(deepestPointInBody0Coordinates, closestSurfacePointOutput, normal))
                )
            }
        }

        return null
    }
}
