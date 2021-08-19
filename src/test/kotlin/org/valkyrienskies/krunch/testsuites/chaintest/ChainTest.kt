package org.valkyrienskies.krunch.testsuites.chaintest

import org.valkyrienskies.krunch.testsuites.OpenGLWindow

fun main(args: Array<String>) {
    val openGLWindow = OpenGLWindow(PhysicsWorldChainTest())
    openGLWindow.run()
}
