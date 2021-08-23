package org.valkyrienskies.krunch.collision.colliders

import org.joml.Matrix4d
import org.joml.Quaterniond
import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Pose
import org.valkyrienskies.krunch.collision.CollisionPair
import org.valkyrienskies.krunch.collision.CollisionPairc
import org.valkyrienskies.krunch.collision.CollisionResult
import org.valkyrienskies.krunch.collision.CollisionResultc
import org.valkyrienskies.krunch.collision.shapes.TSDFVoxelShape

object TSDFVoxelTSDFVoxelCollider : Collider<TSDFVoxelShape, TSDFVoxelShape> {
    override fun computeCollisionBetweenShapes(
        body0Shape: TSDFVoxelShape, body0Transform: Pose, body1Shape: TSDFVoxelShape, body1Transform: Pose
    ): CollisionResultc {
        val collisionPairs = ArrayList<CollisionPairc>()

        val body0VoxelSpaceToLocalCoordinates =
            Matrix4d().scale(body0Shape.scalingFactor).translate(body0Shape.voxelOffset)
        val body1VoxelSpaceToLocalCoordinates =
            Matrix4d().scale(body1Shape.scalingFactor).translate(body1Shape.voxelOffset)

        val body1VoxelSpaceToBody0VoxelSpaceTransform = Matrix4d()

        body1VoxelSpaceToBody0VoxelSpaceTransform.translate(
            -body0Shape.voxelOffset.x, -body0Shape.voxelOffset.y, -body0Shape.voxelOffset.z
        )
        body1VoxelSpaceToBody0VoxelSpaceTransform.scale(1.0 / body0Shape.scalingFactor)
        body1VoxelSpaceToBody0VoxelSpaceTransform.rotate(
            Quaterniond(body0Transform.q).invert()
        )
        body1VoxelSpaceToBody0VoxelSpaceTransform.translate(
            body1Transform.p.x() - body0Transform.p.x(),
            body1Transform.p.y() - body0Transform.p.y(),
            body1Transform.p.z() - body0Transform.p.z()
        )
        body1VoxelSpaceToBody0VoxelSpaceTransform.rotate(body1Transform.q)
        body1VoxelSpaceToBody0VoxelSpaceTransform.scale(body1Shape.scalingFactor)
        body1VoxelSpaceToBody0VoxelSpaceTransform.translate(
            body1Shape.voxelOffset.x, body1Shape.voxelOffset.y, body1Shape.voxelOffset.z
        )

        body1Shape.layeredTSDF.forEachVoxel { posX, posY, posZ ->
            forEachCorner { xCorner: Int, yCorner: Int, zCorner: Int ->
                val pointPosInBody1Coordinates: Vector3dc = Vector3d(
                    posX + xCorner * .25,
                    posY + yCorner * .25,
                    posZ + zCorner * .25
                )
                val pointPosInBodyVoxelSpace0: Vector3dc =
                    body1VoxelSpaceToBody0VoxelSpaceTransform.transformPosition(pointPosInBody1Coordinates, Vector3d())

                val pointSphereRadius = .25

                val closestSurfacePointOutput = Vector3d()

                val wasClosestSurfacePointFound = body0Shape.layeredTSDF.getClosestPoint(
                    pointPosInBodyVoxelSpace0.x(), pointPosInBodyVoxelSpace0.y(), pointPosInBodyVoxelSpace0.z(),
                    closestSurfacePointOutput
                )

                if (wasClosestSurfacePointFound) {
                    val distanceToClosestSurfacePoint = pointPosInBodyVoxelSpace0.distance(closestSurfacePointOutput)

                    if (distanceToClosestSurfacePoint < pointSphereRadius) {
                        val collisionNormalOutput = Vector3d(pointPosInBodyVoxelSpace0).sub(closestSurfacePointOutput)

                        if (collisionNormalOutput.lengthSquared() < 1e-12) {
                            // Avoid numerical instability
                            return@forEachCorner
                        }

                        if (body0Shape.layeredTSDF.getVoxel(
                                pointPosInBodyVoxelSpace0.x(), pointPosInBodyVoxelSpace0.y(),
                                pointPosInBodyVoxelSpace0.z()
                            )
                        ) {
                            collisionNormalOutput.mul(-1.0)
                        }
                        collisionNormalOutput.normalize()

                        val body1CollisionPointInBody0Coordinates =
                            body0VoxelSpaceToLocalCoordinates.transformPosition(
                                Vector3d(pointPosInBodyVoxelSpace0).fma(-pointSphereRadius, collisionNormalOutput)
                            )

                        val body0CollisionPointInBody0Coordinates =
                            body0VoxelSpaceToLocalCoordinates.transformPosition(Vector3d(closestSurfacePointOutput))

                        val normalInGlobalCoordinates = body0Transform.rotate(Vector3d(collisionNormalOutput))

                        val body1CollisionPointInBody1Coordinates = body1Transform.invTransform(
                            body0Transform.transform(
                                Vector3d(body1CollisionPointInBody0Coordinates)
                            )
                        )

                        val collisionPair = CollisionPair(
                            body0CollisionPointInBody0Coordinates, body1CollisionPointInBody1Coordinates,
                            normalInGlobalCoordinates
                        )

                        collisionPairs.add(collisionPair)
                    }
                }
            }
        }

        return CollisionResult(collisionPairs)
    }

    private inline fun forEachCorner(function: (xCorner: Int, yCorner: Int, zCorner: Int) -> Unit) {
        for (i in 0 until 8) {
            val xCorner = if ((i shr 2) and 0x1 == 0x1) -1 else 1
            val yCorner = if ((i shr 1) and 0x1 == 0x1) -1 else 1
            val zCorner = if (i and 0x1 == 0x1) -1 else 1
            function(xCorner, yCorner, zCorner)
        }
        // Equivalent to the following, but much smaller bytecode
        /*
        function(1, 1, 1)
        function(1, 1, -1)
        function(1, -1, 1)
        function(1, -1, -1)
        function(-1, 1, 1)
        function(-1, 1, -1)
        function(-1, -1, 1)
        function(-1, -1, -1)
         */
    }
}
