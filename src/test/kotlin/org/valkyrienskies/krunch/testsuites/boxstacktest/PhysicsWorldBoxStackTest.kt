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

        val groundPose = Pose(Vector3d(0.0, 0.0, 0.0), Quaterniond().rotateAxis(Math.toRadians(0.0), 0.0, 1.0, 1.0))
        val groundBody = Body(groundPose)
        groundBody.setBox(boxSize)
        groundBody.shape = TSDFVoxelShape.createNewVoxelShape(groundBodyVoxels)
        groundBody.isStatic = true

        // endregion

        bodies.add(groundBody)

        for (i in 1..100) {
            val boxPose = Pose(Vector3d(0.0, i.toDouble() + 1, 0.0), Quaterniond())
            val boxBody = Body(boxPose)
            boxBody.setBox(boxSize)
            boxBody.shape = singleVoxelShape
            bodies.add(boxBody)
        }
    }
}
