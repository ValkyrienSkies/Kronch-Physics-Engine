package org.valkyrienskies.krunch.testsuites.boxstacktest

import org.joml.Quaterniond
import org.joml.Vector3d
import org.joml.Vector3i
import org.joml.Vector3ic
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.PhysicsWorld
import org.valkyrienskies.krunch.Pose
import org.valkyrienskies.krunch.collision.shapes.TSDFVoxelShape

class PhysicsWorldBoxStackTest : PhysicsWorld() {

    init {
        // region Create bodies
        val boxSize = Vector3d(1.0, 1.0, 1.0)

        val groundBodyVoxels = ArrayList<Vector3ic>()
        for (x in -10..10) {
            for (z in -10..10) {
                groundBodyVoxels.add(Vector3i(x, 0, z))
            }
        }

        for (x in -2..2) {
            for (z in -2..2) {
                groundBodyVoxels.add(Vector3i(x, 1, z))
            }
        }

        val singleVoxelShape = TSDFVoxelShape.createNewVoxelShape(listOf(Vector3i()))

        val firstBoxPose = Pose(Vector3d(0.0, 2.0, 0.0), Quaterniond())
        val firstBoxBody = Body(firstBoxPose)
        firstBoxBody.setBox(boxSize)
        firstBoxBody.shape = singleVoxelShape

        val secondBoxPose = Pose(Vector3d(0.0, 3.0, 0.0), Quaterniond())
        val secondBoxBody = Body(secondBoxPose)
        secondBoxBody.setBox(boxSize)
        secondBoxBody.shape = singleVoxelShape

        val thirdBoxPose = Pose(Vector3d(0.0, 4.0, 0.0), Quaterniond())
        val thirdBoxBody = Body(thirdBoxPose)
        thirdBoxBody.setBox(boxSize)
        thirdBoxBody.shape = singleVoxelShape

        val fourthBoxPose = Pose(Vector3d(0.3, 20.0, 0.0), Quaterniond())
        val fourthBoxBody = Body(fourthBoxPose)
        fourthBoxBody.setBox(boxSize)
        fourthBoxBody.shape = singleVoxelShape

        val groundPose = Pose(Vector3d(0.0, 0.0, 0.0), Quaterniond().rotateAxis(Math.toRadians(0.0), 0.0, 1.0, 1.0))
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
    }
}
