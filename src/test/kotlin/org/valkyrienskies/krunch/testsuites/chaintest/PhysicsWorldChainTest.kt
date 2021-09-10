package org.valkyrienskies.krunch.testsuites.chaintest

import org.joml.Quaterniond
import org.joml.Vector3d
import org.joml.Vector3i
import org.joml.Vector3ic
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.Joint
import org.valkyrienskies.krunch.JointType.HINGE
import org.valkyrienskies.krunch.JointType.SPHERICAL
import org.valkyrienskies.krunch.PhysicsWorld
import org.valkyrienskies.krunch.Pose
import org.valkyrienskies.krunch.collision.shapes.TSDFVoxelShape
import kotlin.math.abs

class PhysicsWorldChainTest : PhysicsWorld() {

    init {
        // region Create bodies
        val boxSize = Vector3d(1.0, 1.0, 1.0)

        val groundBodyVoxels = ArrayList<Vector3ic>()
        for (x in -10..10) {
            for (z in -10..10) {
                if ((abs(x) == 10) or (abs(z) == 10)) {
                    for (y in 0..10) {
                        groundBodyVoxels.add(Vector3i(x, y, z))
                    }
                }
                groundBodyVoxels.add(Vector3i(x, 0, z))
            }
        }

        // for (x in -2..2) {
        //     for (z in -2..2) {
        //         groundBodyVoxels.add(Vector3i(x, 1, z))
        //     }
        // }

        // groundBodyVoxels.add(Vector3i(0, 2, 0))

        val singleVoxelShape = TSDFVoxelShape.createNewVoxelShape(listOf(Vector3i()))

        val biggerShapeVoxels = ArrayList<Vector3ic>()

        for (x in -1..1) {
            for (z in -1..1) {
                biggerShapeVoxels.add(Vector3i(x, 0, z))
            }
        }
        biggerShapeVoxels.add(Vector3i(0, -1, 0))

        val biggerVoxelShape = TSDFVoxelShape.createNewVoxelShape(biggerShapeVoxels)

        val firstBoxPose = Pose(Vector3d(0.0, 4.5, 0.0), Quaterniond())
        val firstBoxBody = Body(firstBoxPose)
        firstBoxBody.setBox(boxSize)
        firstBoxBody.shape = singleVoxelShape

        val secondBoxPose = Pose(Vector3d(2.0, 8.5, 0.0), Quaterniond())
        val secondBoxBody = Body(secondBoxPose)
        secondBoxBody.setBox(boxSize)
        secondBoxBody.shape = biggerVoxelShape

        val thirdBoxPose = Pose(Vector3d(5.0, 8.5, 0.0), Quaterniond())
        val thirdBoxBody = Body(thirdBoxPose)
        thirdBoxBody.setBox(boxSize)
        thirdBoxBody.shape = biggerVoxelShape

        val groundPose = Pose(Vector3d(0.0, 0.0, 0.0), Quaterniond().rotateAxis(Math.toRadians(10.0), 0.0, 1.0, 1.0))
        val groundBody = Body(groundPose)
        groundBody.setBox(boxSize)
        groundBody.shape = TSDFVoxelShape.createNewVoxelShape(groundBodyVoxels)
        groundBody.isStatic = true

        // endregion

        // region Create joins
        val firstBoxToCeilingJoint =
            Joint(
                SPHERICAL, null, firstBoxBody, Pose(Vector3d(0.0, 4.5, 0.0), Quaterniond()),
                Pose(Vector3d(0.5, .5, 0.5), Quaterniond())
            )

        val firstBoxToSecondBoxJoint =
            Joint(
                HINGE, firstBoxBody, secondBoxBody, Pose(Vector3d(-0.5, -.5, -0.5), Quaterniond()),
                Pose(Vector3d(0.5, -1.5, 0.5), Quaterniond())
            )

        val secondBoxToThirdBoxJoint =
            Joint(
                HINGE, secondBoxBody, thirdBoxBody, Pose(Vector3d(.5, .5, .5), Quaterniond()),
                Pose(Vector3d(0.5, -1.5, 0.5), Quaterniond())
            )

        // Add damping forces to the joints
        val jointRotDamping = 1.0
        val jointPosDamping = 1.0

        firstBoxToCeilingJoint.rotDamping = jointRotDamping
        firstBoxToCeilingJoint.posDamping = jointPosDamping

        firstBoxToSecondBoxJoint.rotDamping = jointRotDamping
        firstBoxToSecondBoxJoint.posDamping = jointPosDamping

        secondBoxToThirdBoxJoint.rotDamping = jointRotDamping
        secondBoxToThirdBoxJoint.posDamping = jointPosDamping

        // endregion
        bodies.add(groundBody)
        bodies.add(firstBoxBody)
        bodies.add(secondBoxBody)
        bodies.add(thirdBoxBody)

        joints.add(firstBoxToCeilingJoint)
        joints.add(firstBoxToSecondBoxJoint)
        joints.add(secondBoxToThirdBoxJoint)

        for (i in 1..5) {
            val secondBoxPose = Pose(Vector3d(-1.5, 8.5 + i * 5, 0.0), Quaterniond())
            val secondBoxBody = Body(secondBoxPose)
            secondBoxBody.setBox(boxSize)
            secondBoxBody.shape = biggerVoxelShape

            val thirdBoxPose = Pose(Vector3d(1.5, 8.5 + i * 5, 0.0), Quaterniond())
            val thirdBoxBody = Body(thirdBoxPose)
            thirdBoxBody.setBox(boxSize)
            thirdBoxBody.shape = biggerVoxelShape

            val secondBoxToThirdBoxJoint =
                Joint(
                    HINGE, secondBoxBody, thirdBoxBody, Pose(Vector3d(.5, .5, .5), Quaterniond()),
                    Pose(Vector3d(0.5, -1.5, 0.5), Quaterniond())
                )

            bodies.add(secondBoxBody)
            bodies.add(thirdBoxBody)
            joints.add(secondBoxToThirdBoxJoint)
        }
    }
}
