package org.valkyrienskies.krunch.testsuites.boxstacktest

import org.joml.Vector3f
import org.valkyrienskies.krunch.testsuites.OpenGLWindow

// A class isn't necessary here, but makes this easy to lookup in IntelliJ
class BoxStackTest

fun main(args: Array<String>) {
    val openGLWindow = OpenGLWindow(PhysicsWorldBoxStackTest(), cameraEyePos = Vector3f(0.0f, 8.0f, 15.0f))
    openGLWindow.run()
}
