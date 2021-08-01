package org.valkyrienskies.krunch.testsuites

fun main(args: Array<String>) {
    OpenGLTestSuite.main(args)
}

internal object OpenGLTestSuite {
    fun main(args: Array<String>) {
        val helloWorld = OpenGLWindow()
        helloWorld.run()
    }
}
