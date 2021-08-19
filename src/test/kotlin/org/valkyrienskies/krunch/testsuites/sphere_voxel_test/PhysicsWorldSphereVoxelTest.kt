package org.valkyrienskies.krunch.testsuites.sphere_voxel_test

import org.joml.Quaterniond
import org.joml.Vector3d
import org.joml.Vector3i
import org.joml.Vector3ic
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.PhysicsWorld
import org.valkyrienskies.krunch.Pose
import org.valkyrienskies.krunch.collision.shapes.SphereShape
import org.valkyrienskies.krunch.collision.shapes.TSDFVoxelShape
import kotlin.math.abs

class PhysicsWorldSphereVoxelTest : PhysicsWorld() {

    init {
        // region Create bodies
        val boxSize = Vector3d(1.0, 1.0, 1.0)

        val groundBodyVoxels = ArrayList<Vector3ic>()
        for (x in -10..10) {
            for (z in -10..10) {
                groundBodyVoxels.add(Vector3i(x, 0, z))
            }
        }

        for (x in -3..3) {
            for (z in -3..3) {
                if ((abs(x) == 3) or (abs(z) == 3))
                    groundBodyVoxels.add(Vector3i(x, 1, z))
            }
        }

        val bigCubeVoxels = ArrayList<Vector3ic>()
        for (x in -1..1) {
            for (y in -1..1) {
                for (z in -1..1) {
                    bigCubeVoxels.add(Vector3i(x, y, z))
                }
            }
        }
        val bigCubeVoxelShape = TSDFVoxelShape.createNewVoxelShape(bigCubeVoxels)

        val sphereShape = SphereShape(.5)

        val firstBoxPose = Pose(Vector3d(0.0, 2.0, 0.0), Quaterniond())
        val firstBoxBody = Body(firstBoxPose)
        firstBoxBody.setBox(boxSize)
        firstBoxBody.shape = sphereShape

        val secondBoxPose = Pose(Vector3d(0.0, 3.0, 0.0), Quaterniond())
        val secondBoxBody = Body(secondBoxPose)
        secondBoxBody.setBox(boxSize)
        secondBoxBody.shape = sphereShape

        val thirdBoxPose = Pose(Vector3d(0.0, 4.0, 0.0), Quaterniond())
        val thirdBoxBody = Body(thirdBoxPose)
        thirdBoxBody.setBox(boxSize)
        thirdBoxBody.shape = sphereShape

        val fourthBoxPose = Pose(Vector3d(0.1, 20.0, 0.0), Quaterniond())
        val fourthBoxBody = Body(fourthBoxPose)
        fourthBoxBody.setBox(boxSize)
        fourthBoxBody.shape = sphereShape

        val fifthBodyPose = Pose(Vector3d(0.0, 30.0, 0.0), Quaterniond())
        val fifthBody = Body(fifthBodyPose)
        fifthBody.setBox(boxSize)
        fifthBody.shape = bigCubeVoxelShape

        val groundPose = Pose(Vector3d(0.0, 0.0, 0.0), Quaterniond().rotateAxis(Math.toRadians(20.0), 0.0, 1.0, 1.0))
        val groundBody = Body(groundPose)
        groundBody.setBox(boxSize)
        groundBody.shape = TSDFVoxelShape.createNewVoxelShape(groundBodyVoxels)
        groundBody.isStatic = true

        // endregion

        bodies.add(groundBody)
        bodies.add(firstBoxBody)
        bodies.add(secondBoxBody)
        bodies.add(thirdBoxBody)
        bodies.add(fourthBoxBody)
        bodies.add(fifthBody)
    }

    override fun simulate(timeStep: Double) {
        val gravity = Vector3d(0.0, -10.0, 0.0)
        val numSubsteps = 40
        org.valkyrienskies.krunch.simulate(bodies, joints, timeStep, numSubsteps, gravity)
    }
}
