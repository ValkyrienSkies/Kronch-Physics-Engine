package org.valkyrienskies.krunch.collision.colliders

import org.joml.Matrix4d
import org.joml.Quaterniond
import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Posec
import org.valkyrienskies.krunch.collision.CollisionPair
import org.valkyrienskies.krunch.collision.CollisionResult
import org.valkyrienskies.krunch.collision.shapes.TSDFVoxelShape
import org.valkyrienskies.krunch.computeRelativeVelocityBetweenCollisionPointsAlongNormal
import kotlin.math.abs

object TSDFVoxelTSDFVoxelCollider : Collider<TSDFVoxelShape, TSDFVoxelShape> {
    override fun computeCollisionBetweenShapes(
        body0Shape: TSDFVoxelShape,
        body0Transform: Posec,
        body0Velocity: Vector3dc,
        body0AngularVelocity: Vector3dc,
        body1Shape: TSDFVoxelShape,
        body1Transform: Posec,
        body1Velocity: Vector3dc,
        body1AngularVelocity: Vector3dc,
        dt: Double,
        speculativeThreshold: Double
    ): CollisionResult {
        val collisionPairs = ArrayList<CollisionPair>()

        val body0VoxelSpaceToLocalCoordinates =
            Matrix4d().scale(body0Shape.scalingFactor).translate(body0Shape.voxelOffset)

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

                // Since we are going from body 1 voxel space to body 0 voxel space, we must scale the point size by the relative scaling factor
                val pointSphereRadius = abs(.25 * body1Shape.scalingFactor / body0Shape.scalingFactor)

                val closestSurfacePointOutput = Vector3d()

                val wasClosestSurfacePointFound = body0Shape.layeredTSDF.getClosestPoint(
                    pointPosInBodyVoxelSpace0.x(), pointPosInBodyVoxelSpace0.y(), pointPosInBodyVoxelSpace0.z(),
                    closestSurfacePointOutput
                )

                if (wasClosestSurfacePointFound) {
                    val distanceToClosestSurfacePoint = pointPosInBodyVoxelSpace0.distance(closestSurfacePointOutput)

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

                    // If body0Shape has negative scaling, then flip the normal
                    if (body0Shape.scalingFactor < 0) {
                        normalInGlobalCoordinates.mul(-1.0)
                    }

                    val body1CollisionPointInBody1Coordinates = body1Transform.invTransform(
                        body0Transform.transform(
                            Vector3d(body1CollisionPointInBody0Coordinates)
                        )
                    )

                    val collisionPair = CollisionPair(
                        body0CollisionPointInBody0Coordinates, body1CollisionPointInBody1Coordinates,
                        normalInGlobalCoordinates
                    )

                    val relativeVelocity = computeRelativeVelocityBetweenCollisionPointsAlongNormal(
                        normalInGlobalCoordinates,
                        body0CollisionPointInBody0Coordinates,
                        body1CollisionPointInBody1Coordinates,
                        body0Transform,
                        body0Velocity,
                        body0AngularVelocity,
                        body1Transform,
                        body1Velocity,
                        body1AngularVelocity
                    ) * dt

                    // Since we are in body1 voxel space, scale the speculative contract and relative velocity to go
                    // from world coordinates to body1 voxel coordinates
                    val speculativeContactsThresholdAndRelativeVelocity =
                        (relativeVelocity - speculativeThreshold) / body1Shape.scalingFactor

                    // This check uses distances relative to body1 size
                    if (distanceToClosestSurfacePoint + speculativeContactsThresholdAndRelativeVelocity < pointSphereRadius) {
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
