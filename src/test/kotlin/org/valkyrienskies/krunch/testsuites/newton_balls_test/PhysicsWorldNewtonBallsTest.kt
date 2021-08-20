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

        val firstBoxPose = Pose(Vector3d(0.0, 3.0, 0.0), Quaterniond())
        val firstBallBody = Body(firstBoxPose)
        firstBallBody.setBox(boxSize)
        firstBallBody.shape = sphereShape

        val secondBoxPose = Pose(Vector3d(4.0, 4.0, 0.0), Quaterniond())
        val secondBallBody = Body(secondBoxPose)
        secondBallBody.setBox(boxSize)
        secondBallBody.shape = sphereShape

        val thirdBoxPose = Pose(Vector3d(0.0, 4.0, 0.0), Quaterniond())
        val thirdBoxBody = Body(thirdBoxPose)
        thirdBoxBody.setBox(boxSize)
        thirdBoxBody.shape = sphereShape

        val groundPose = Pose(Vector3d(0.0, 0.0, 0.0), Quaterniond().rotateAxis(Math.toRadians(0.0), 0.0, 1.0, 1.0))
        val groundBody = Body(groundPose)
        groundBody.setBox(boxSize)
        groundBody.shape = TSDFVoxelShape.createNewVoxelShape(groundBodyVoxels)
        groundBody.isStatic = true

        val firstRailSegmentBody = Body.createStaticBody(
            Pose(Vector3d(0.0, 2.0, 0.3)),
            BoxShape(3.0, .1, .1)
        )
        val secondRailSegmentBody = Body.createStaticBody(
            Pose(Vector3d(3.75, 2.35, 0.3), Quaterniond().rotateAxis(Math.toRadians(10.0), 0.0, 0.0, 1.0)),
            BoxShape(2.0, .1, .1)
        )
        val thirdRailSegmentBody = Body.createStaticBody(
            Pose(Vector3d(-3.75, 2.35, 0.3), Quaterniond().rotateAxis(Math.toRadians(-10.0), 0.0, 0.0, 1.0)),
            BoxShape(2.0, .1, .1)
        )
        val fourthRailSegmentBody = Body.createStaticBody(
            Pose(Vector3d(0.0, 2.0, -0.3)),
            BoxShape(3.0, .1, .1)
        )
        val fifthRailSegmentBody = Body.createStaticBody(
            Pose(Vector3d(3.75, 2.35, -0.3), Quaterniond().rotateAxis(Math.toRadians(10.0), 0.0, 0.0, 1.0)),
            BoxShape(2.0, .1, .1)
        )
        val sixthRailSegmentBody = Body.createStaticBody(
            Pose(Vector3d(-3.75, 2.35, -0.3), Quaterniond().rotateAxis(Math.toRadians(-10.0), 0.0, 0.0, 1.0)),
            BoxShape(2.0, .1, .1)
        )
        // endregion

        bodies.add(groundBody)
        bodies.add(firstRailSegmentBody)
        bodies.add(secondRailSegmentBody)
        bodies.add(thirdRailSegmentBody)
        bodies.add(fourthRailSegmentBody)
        bodies.add(fifthRailSegmentBody)
        bodies.add(sixthRailSegmentBody)

        bodies.add(firstBallBody)
        bodies.add(secondBallBody)
        // bodies.add(thirdBoxBody)
    }

    override fun simulate(timeStep: Double) {
        val gravity = Vector3d(-1.0, -10.0, 0.0)
        val numSubsteps = 40
        org.valkyrienskies.krunch.simulate(bodies, joints, timeStep, numSubsteps, gravity)
    }
}
