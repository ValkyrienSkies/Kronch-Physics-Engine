package org.valkyrienskies.krunch.testsuites.voxel_shape_offset_test

import org.joml.Vector3f
import org.valkyrienskies.krunch.testsuites.OpenGLWindow

// A class isn't necessary here, but makes this easy to lookup in IntelliJ
class VoxelShapeOffsetTest

fun main(args: Array<String>) {
    val openGLWindow = OpenGLWindow(PhysicsWorldVoxelShapeOffsetTest(), cameraEyePos = Vector3f(0.0f, 8.0f, 15.0f))
    openGLWindow.renderCenterOfBodies = true
    openGLWindow.run()
}
