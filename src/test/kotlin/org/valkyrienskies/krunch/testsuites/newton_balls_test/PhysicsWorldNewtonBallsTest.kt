package org.valkyrienskies.krunch.testsuites.newton_balls_test

import org.joml.Quaterniond
import org.joml.Vector3d
import org.joml.Vector3i
import org.joml.Vector3ic
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.PhysicsWorld
import org.valkyrienskies.krunch.Pose
import org.valkyrienskies.krunch.collision.shapes.BoxShape
import org.valkyrienskies.krunch.collision.shapes.SphereShape
import org.valkyrienskies.krunch.collision.shapes.TSDFVoxelShape

class PhysicsWorldNewtonBallsTest : PhysicsWorld() {

    init {
        // region Create bodies
        val boxSize = Vector3d(1.0, 1.0, 1.0)

        val groundBodyVoxels = ArrayList<Vector3ic>()
        for (x in -10..10) {
            for (z in -10..10) {
                groundBodyVoxels.add(Vector3i(x, 0, z))
            }
        }

        val sphereShape = SphereShape(.5)

        val firstBallBody = Body(Pose(Vector3d(0.0, 2.5, 0.0), Quaterniond()))
        firstBallBody.setBox(boxSize)
        firstBallBody.shape = sphereShape

        val secondBallBody = Body(Pose(Vector3d(1.0, 2.5, 0.0), Quaterniond()))
        secondBallBody.setBox(boxSize)
        secondBallBody.shape = sphereShape

        val thirdBallBody = Body(Pose(Vector3d(-1.0, 2.5, 0.0), Quaterniond()))
        thirdBallBody.setBox(boxSize)
        thirdBallBody.shape = sphereShape

        val fourthBallBody = Body(Pose(Vector3d(6.0, 5.0, 0.0), Quaterniond()))
        fourthBallBody.setBox(boxSize)
        fourthBallBody.shape = sphereShape

        val groundPose = Pose(Vector3d(0.0, 0.0, 0.0), Quaterniond().rotateAxis(Math.toRadians(0.0), 0.0, 1.0, 1.0))
        val groundBody = Body(groundPose)
        groundBody.setBox(boxSize)
        groundBody.shape = TSDFVoxelShape.createNewVoxelShape(groundBodyVoxels)
        groundBody.isStatic = true

        val railDistanceFromOrigin = .45

        val firstRailSegmentBody = Body.createStaticBody(
            Pose(Vector3d(0.0, 2.0, railDistanceFromOrigin)),
            BoxShape(3.0, .1, .1)
        )
        val secondRailSegmentBody = Body.createStaticBody(
            Pose(
                Vector3d(3.75, 2.5, railDistanceFromOrigin),
                Quaterniond().rotateAxis(Math.toRadians(15.0), 0.0, 0.0, 1.0)
            ),
            BoxShape(2.5, .1, .1)
        )
        val thirdRailSegmentBody = Body.createStaticBody(
            Pose(
                Vector3d(-3.75, 2.5, railDistanceFromOrigin),
                Quaterniond().rotateAxis(Math.toRadians(-15.0), 0.0, 0.0, 1.0)
            ),
            BoxShape(2.5, .1, .1)
        )
        val fourthRailSegmentBody = Body.createStaticBody(
            Pose(
                Vector3d(-5.0, 3.25, railDistanceFromOrigin),
                Quaterniond().rotateAxis(Math.toRadians(-30.0), 0.0, 0.0, 1.0)
            ),
            BoxShape(2.0, .1, .1)
        )
        val fifthRailSegmentBody = Body.createStaticBody(
            Pose(
                Vector3d(5.0, 3.25, railDistanceFromOrigin),
                Quaterniond().rotateAxis(Math.toRadians(30.0), 0.0, 0.0, 1.0)
            ),
            BoxShape(2.0, .1, .1)
        )

        val firstRailSegmentBodyOpposite = Body.createStaticBody(
            Pose(Vector3d(0.0, 2.0, -railDistanceFromOrigin)),
            BoxShape(3.0, .1, .1)
        )
        val secondRailSegmentBodyOpposite = Body.createStaticBody(
            Pose(
                Vector3d(3.75, 2.5, -railDistanceFromOrigin),
                Quaterniond().rotateAxis(Math.toRadians(15.0), 0.0, 0.0, 1.0)
            ),
            BoxShape(2.5, .1, .1)
        )
        val thirdRailSegmentBodyOpposite = Body.createStaticBody(
            Pose(
                Vector3d(-3.75, 2.5, -railDistanceFromOrigin),
                Quaterniond().rotateAxis(Math.toRadians(-15.0), 0.0, 0.0, 1.0)
            ),
            BoxShape(2.5, .1, .1)
        )
        val fourthRailSegmentBodyOpposite = Body.createStaticBody(
            Pose(
                Vector3d(-5.0, 3.25, -railDistanceFromOrigin),
                Quaterniond().rotateAxis(Math.toRadians(-30.0), 0.0, 0.0, 1.0)
            ),
            BoxShape(2.0, .1, .1)
        )
        val fifthRailSegmentBodyOpposite = Body.createStaticBody(
            Pose(
                Vector3d(5.0, 3.25, -railDistanceFromOrigin),
                Quaterniond().rotateAxis(Math.toRadians(30.0), 0.0, 0.0, 1.0)
            ),
            BoxShape(2.0, .1, .1)
        )
        // endregion

        bodies.add(groundBody)

        bodies.add(firstRailSegmentBody)
        bodies.add(secondRailSegmentBody)
        bodies.add(thirdRailSegmentBody)
        bodies.add(fourthRailSegmentBody)
        bodies.add(fifthRailSegmentBody)

        bodies.add(firstRailSegmentBodyOpposite)
        bodies.add(secondRailSegmentBodyOpposite)
        bodies.add(thirdRailSegmentBodyOpposite)
        bodies.add(fourthRailSegmentBodyOpposite)
        bodies.add(fifthRailSegmentBodyOpposite)

        bodies.add(firstBallBody)
        bodies.add(secondBallBody)
        bodies.add(thirdBallBody)
        bodies.add(fourthBallBody)
    }

    override fun simulate(timeStep: Double) {
        val gravity = Vector3d(0.0, -10.0, 0.0)
        val numSubsteps = 40
        org.valkyrienskies.krunch.simulate(bodies, joints, timeStep, numSubsteps, gravity)
    }
}
