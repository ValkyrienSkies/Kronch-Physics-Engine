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
import org.valkyrienskies.krunch.collision.shapes.NewVoxelShape

object NewVoxelNewVoxelCollider : Collider<NewVoxelShape, NewVoxelShape> {
    override fun computeCollisionBetweenShapes(
        body0Shape: NewVoxelShape, body0Transform: Pose, body1Shape: NewVoxelShape, body1Transform: Pose
    ): CollisionResultc {
        val collisionPairs = ArrayList<CollisionPairc>()

        val body1To0Transform = Matrix4d()

        body1To0Transform.translate(
            -body0Shape.shapeOffset.x, -body0Shape.shapeOffset.y, -body0Shape.shapeOffset.z
        )
        body1To0Transform.rotate(
            Quaterniond(body0Transform.q).invert()
        )
        body1To0Transform.translate(
            body1Transform.p.x() - body0Transform.p.x(),
            body1Transform.p.y() - body0Transform.p.y(),
            body1Transform.p.z() - body0Transform.p.z()
        )
        body1To0Transform.rotate(body1Transform.q)
        body1To0Transform.translate(
            body1Shape.shapeOffset.x, body1Shape.shapeOffset.y, body1Shape.shapeOffset.z
        )

        body1Shape.layeredTSDF.forEachVoxel { posX, posY, posZ ->
            forEachCorner { xCorner: Int, yCorner: Int, zCorner: Int ->
                val pointPosInBody1Coordinates: Vector3dc = Vector3d(
                    posX + xCorner * .25,
                    posY + yCorner * .25,
                    posZ + zCorner * .25
                )
                val pointPosInBody0Coordinates: Vector3dc =
                    body1To0Transform.transformPosition(pointPosInBody1Coordinates, Vector3d())

                val pointSphereRadius = .25

                val closestSurfacePointOutput = Vector3d()

                val wasClosestSurfacePointFound = body0Shape.layeredTSDF.getClosestPoint(
                    pointPosInBody0Coordinates.x(), pointPosInBody0Coordinates.y(), pointPosInBody0Coordinates.z(),
                    closestSurfacePointOutput
                )

                if (wasClosestSurfacePointFound) {
                    val distanceToClosestSurfacePoint = pointPosInBody0Coordinates.distance(closestSurfacePointOutput)

                    if (distanceToClosestSurfacePoint < pointSphereRadius) {
                        // Collision
                        // TODO: Figure out this part :|
                        val collisionNormalOutput = Vector3d(pointPosInBody0Coordinates).sub(closestSurfacePointOutput)
                        if (body0Shape.layeredTSDF.getVoxel(
                                pointPosInBody0Coordinates.x(), pointPosInBody0Coordinates.y(),
                                pointPosInBody0Coordinates.z()
                            )
                        ) collisionNormalOutput.mul(-1.0)
                        collisionNormalOutput as Vector3dc

                        val body1CollisionPointInBody0Coordinates =
                            Vector3d(pointPosInBody0Coordinates).fma(-.25, collisionNormalOutput)
                        val body0CollisionPointInBody0Coordinates = Vector3d(closestSurfacePointOutput)

                        val normalInGlobalCoordinates = body0Transform.rotate(Vector3d(collisionNormalOutput)).mul(-1.0)

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
