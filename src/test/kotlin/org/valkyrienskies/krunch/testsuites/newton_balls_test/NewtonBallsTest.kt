package org.valkyrienskies.krunch.testsuites.newton_balls_test

import org.joml.Vector3f
import org.valkyrienskies.krunch.testsuites.OpenGLWindow

// A class isn't necessary here, but makes this easy to lookup in IntelliJ
class NewtonBallsTest

fun main(args: Array<String>) {
    val openGLWindow = OpenGLWindow(PhysicsWorldNewtonBallsTest(), cameraEyePos = Vector3f(0.0f, 8.0f, 12.0f))
    openGLWindow.run()
}
