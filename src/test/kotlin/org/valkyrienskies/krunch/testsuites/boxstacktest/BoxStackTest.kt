import org.valkyrienskies.krunch.testsuites.OpenGLWindow
import org.valkyrienskies.krunch.testsuites.boxstacktest.PhysicsWorldBoxStackTest

fun main(args: Array<String>) {
    val openGLWindow = OpenGLWindow(PhysicsWorldBoxStackTest())
    openGLWindow.run()
}
