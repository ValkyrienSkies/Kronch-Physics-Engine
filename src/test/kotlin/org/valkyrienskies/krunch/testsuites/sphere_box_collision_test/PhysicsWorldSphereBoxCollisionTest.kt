package org.valkyrienskies.krunch.testsuites.sphere_box_collision_test

import org.joml.Quaterniond
import org.joml.Vector3d
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.PhysicsWorld
import org.valkyrienskies.krunch.Pose
import org.valkyrienskies.krunch.collision.shapes.BoxShape
import org.valkyrienskies.krunch.collision.shapes.SphereShape

class PhysicsWorldSphereBoxCollisionTest : PhysicsWorld() {

    init {
        // region Create bodies
        val boxSize = Vector3d(1.0, 1.0, 1.0)

        val sphereShape = SphereShape(.5)

        val firstBallPose = Pose(Vector3d(0.0, 5.0, 0.0), Quaterniond())
        val firstBallBody = Body(firstBallPose)
        firstBallBody.setBox(boxSize)
        firstBallBody.shape = sphereShape
        firstBallBody.coefficientOfRestitution = 1.0

        val secondBallBody = Body(Pose(Vector3d(0.0, 8.0, 0.0), Quaterniond()))
        secondBallBody.setBox(boxSize)
        secondBallBody.shape = sphereShape
        secondBallBody.coefficientOfRestitution = 1.0

        val thirdBallBody = Body(Pose(Vector3d(0.0, 2.0, 0.0), Quaterniond()))
        thirdBallBody.setBox(boxSize)
        thirdBallBody.shape = sphereShape
        thirdBallBody.coefficientOfRestitution = 1.0

        val fourthBallBody = Body(Pose(Vector3d(0.0, 12.0, 0.0), Quaterniond()))
        fourthBallBody.setBox(boxSize)
        fourthBallBody.shape = sphereShape
        fourthBallBody.coefficientOfRestitution = 1.0

        val fifthBallBody = Body(Pose(Vector3d(0.0, 15.0, 0.0), Quaterniond()))
        fifthBallBody.setBox(boxSize)
        fifthBallBody.shape = sphereShape
        fifthBallBody.coefficientOfRestitution = 1.0

        val firstBoxBody = Body.createStaticBody(
            Pose(Vector3d(0.0, 0.0, 0.3), Quaterniond().rotateAxis(Math.toRadians(0.0), 0.0, 0.0, 1.0)),
            BoxShape(10.0, .5, 10.0)
        )
        firstBoxBody.coefficientOfRestitution = 1.0

        val secondBoxBody = Body.createStaticBody(
            Pose(Vector3d(5.0, 1.0, 0.0), Quaterniond().rotateAxis(Math.toRadians(45.0), 0.0, 1.0, 0.0)),
            BoxShape(10.0, .5, .5)
        )
        val thirdBoxBody = Body.createStaticBody(
            Pose(Vector3d(-5.0, 1.0, 0.0), Quaterniond().rotateAxis(Math.toRadians(135.0), 0.0, 1.0, 0.0)),
            BoxShape(10.0, .5, .5)
        )
        // endregion

        bodies.add(firstBoxBody)
        // bodies.add(secondBoxBody)
        // bodies.add(thirdBoxBody)

        bodies.add(firstBallBody)
        bodies.add(secondBallBody)
        bodies.add(thirdBallBody)
        bodies.add(fourthBallBody)
        bodies.add(fifthBallBody)
    }

    override fun simulate(timeStep: Double) {
        val gravity = Vector3d(0.0, -10.0, 0.0)
        val numSubsteps = 40
        org.valkyrienskies.krunch.simulate(bodies, joints, timeStep, numSubsteps, gravity)
    }
}
