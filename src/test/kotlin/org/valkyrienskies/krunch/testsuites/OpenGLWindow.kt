package org.valkyrienskies.krunch.testsuites

import org.joml.AxisAngle4d
import org.joml.Matrix4f
import org.joml.Matrix4x3f
import org.joml.Vector3f
import org.joml.Vector3fc
import org.joml.Vector4f
import org.joml.Vector4fc
import org.lwjgl.BufferUtils
import org.lwjgl.glfw.GLFW
import org.lwjgl.glfw.GLFWErrorCallback
import org.lwjgl.glfw.GLFWFramebufferSizeCallback
import org.lwjgl.glfw.GLFWKeyCallback
import org.lwjgl.opengl.GL
import org.lwjgl.opengl.GL11
import org.lwjgl.system.MemoryUtil
import org.valkyrienskies.krunch.PhysicsWorld
import org.valkyrienskies.krunch.collision.shapes.BoxShape
import org.valkyrienskies.krunch.collision.shapes.SphereShape
import org.valkyrienskies.krunch.collision.shapes.TSDFVoxelShape
import java.nio.FloatBuffer

/**
 * Creates a window that renders using OpenGL. Based off of https://github.com/LWJGL/lwjgl3-demos/blob/79e81f6f3794c3dfd32ba0a31aefa69e56ad603b/src/org/lwjgl/demo/opengl/transform/LwjglDemo.java.
 */
class OpenGLWindow(
    private val physicsWorld: PhysicsWorld,
    private val cameraEyePos: Vector3fc = Vector3f(0.0f, 4.0f, 10.0f),
    private val cameraLookAtPos: Vector3fc = Vector3f(0.0f, 0.0f, 0.0f)
) {
    var errorCallback: GLFWErrorCallback? = null
    var keyCallback: GLFWKeyCallback? = null
    var fbCallback: GLFWFramebufferSizeCallback? = null

    var window: Long = 0
    var width = 400
    var height = 300
    var renderCenterOfBodies = false

    // JOML matrices
    var projMatrix = Matrix4f()
    var viewMatrix = Matrix4x3f()

    // FloatBuffer for transferring matrices to OpenGL
    private var floatBuffer: FloatBuffer = BufferUtils.createFloatBuffer(16)

    fun run() {
        try {
            init()
            loop()
            GLFW.glfwDestroyWindow(window)
            keyCallback!!.free()
        } finally {
            GLFW.glfwTerminate()
            errorCallback!!.free()
        }
    }

    private fun init() {
        GLFW.glfwSetErrorCallback(GLFWErrorCallback.createPrint(System.err).also { errorCallback = it })
        check(GLFW.glfwInit()) { "Unable to initialize GLFW" }

        // Configure our window
        GLFW.glfwDefaultWindowHints()
        GLFW.glfwWindowHint(GLFW.GLFW_VISIBLE, GLFW.GLFW_FALSE)
        GLFW.glfwWindowHint(GLFW.GLFW_RESIZABLE, GLFW.GLFW_TRUE)
        window = GLFW.glfwCreateWindow(width, height, "Krunch Test Suite", MemoryUtil.NULL, MemoryUtil.NULL)
        if (window == MemoryUtil.NULL) throw RuntimeException("Failed to create the GLFW window")
        GLFW.glfwSetKeyCallback(
            window,
            object : GLFWKeyCallback() {
                override fun invoke(window: Long, key: Int, scancode: Int, action: Int, mods: Int) {
                    if (key == GLFW.GLFW_KEY_ESCAPE && action == GLFW.GLFW_RELEASE) GLFW.glfwSetWindowShouldClose(
                        window, true
                    )
                    if (key == GLFW.GLFW_KEY_UP) {
                        for (body in physicsWorld.bodies) {
                            body.vel.add(0.0, 10.0, 0.0)
                        }
                    }
                }
            }.also { keyCallback = it }
        )
        GLFW.glfwSetFramebufferSizeCallback(
            window,
            object : GLFWFramebufferSizeCallback() {
                override fun invoke(window: Long, w: Int, h: Int) {
                    if (w > 0 && h > 0) {
                        width = w
                        height = h
                    }
                }
            }.also { fbCallback = it }
        )
        val vidmode = GLFW.glfwGetVideoMode(GLFW.glfwGetPrimaryMonitor())
        GLFW.glfwSetWindowPos(window, (vidmode!!.width() - width) / 2, (vidmode.height() - height) / 2)
        GLFW.glfwMakeContextCurrent(window)
        GLFW.glfwSwapInterval(0)
        GLFW.glfwShowWindow(window)
    }

    private fun renderCube(xWidth: Float = .5f, yWidth: Float = .5f, zWidth: Float = .5f, color: Vector4fc? = null) {
        val scale = .99f
        val xWidthScaled = xWidth * scale
        val yWidthScaled = yWidth * scale
        val zWidthScaled = zWidth * scale
        GL11.glBegin(GL11.GL_QUADS)

        if (color != null) {
            GL11.glColor4f(color.x(), color.y(), color.z(), color.w())
        } else {
            GL11.glColor4f(0.0f, 0.0f, 0.2f, 1.0f)
        }
        GL11.glVertex3f(xWidthScaled, -yWidthScaled, -zWidthScaled)
        GL11.glVertex3f(-xWidthScaled, -yWidthScaled, -zWidthScaled)
        GL11.glVertex3f(-xWidthScaled, yWidthScaled, -zWidthScaled)
        GL11.glVertex3f(xWidthScaled, yWidthScaled, -zWidthScaled)

        if (color != null) {
            GL11.glColor4f(color.x(), color.y(), color.z(), color.w())
        } else {
            GL11.glColor4f(0.0f, 0.0f, 1.0f, 1.0f)
        }
        GL11.glVertex3f(xWidthScaled, -yWidthScaled, zWidthScaled)
        GL11.glVertex3f(xWidthScaled, yWidthScaled, zWidthScaled)
        GL11.glVertex3f(-xWidthScaled, yWidthScaled, zWidthScaled)
        GL11.glVertex3f(-xWidthScaled, -yWidthScaled, zWidthScaled)

        if (color != null) {
            GL11.glColor4f(color.x(), color.y(), color.z(), color.w())
        } else {
            GL11.glColor3f(1.0f, 0.0f, 0.0f)
        }
        GL11.glVertex3f(xWidthScaled, -yWidthScaled, -zWidthScaled)
        GL11.glVertex3f(xWidthScaled, yWidthScaled, -zWidthScaled)
        GL11.glVertex3f(xWidthScaled, yWidthScaled, zWidthScaled)
        GL11.glVertex3f(xWidthScaled, -yWidthScaled, zWidthScaled)

        if (color != null) {
            GL11.glColor4f(color.x(), color.y(), color.z(), color.w())
        } else {
            GL11.glColor4f(0.2f, 0.0f, 0.0f, 1.0f)
        }
        GL11.glVertex3f(-xWidthScaled, -yWidthScaled, zWidthScaled)
        GL11.glVertex3f(-xWidthScaled, yWidthScaled, zWidthScaled)
        GL11.glVertex3f(-xWidthScaled, yWidthScaled, -zWidthScaled)
        GL11.glVertex3f(-xWidthScaled, -yWidthScaled, -zWidthScaled)

        if (color != null) {
            GL11.glColor4f(color.x(), color.y(), color.z(), color.w())
        } else {
            GL11.glColor4f(0.0f, 1.0f, 0.0f, 1.0f)
        }
        GL11.glVertex3f(xWidthScaled, yWidthScaled, zWidthScaled)
        GL11.glVertex3f(xWidthScaled, yWidthScaled, -zWidthScaled)
        GL11.glVertex3f(-xWidthScaled, yWidthScaled, -zWidthScaled)
        GL11.glVertex3f(-xWidthScaled, yWidthScaled, zWidthScaled)

        if (color != null) {
            GL11.glColor4f(color.x(), color.y(), color.z(), color.w())
        } else {
            GL11.glColor4f(0.0f, 0.2f, 0.0f, 1.0f)
        }
        GL11.glVertex3f(xWidthScaled, -yWidthScaled, -zWidthScaled)
        GL11.glVertex3f(xWidthScaled, -yWidthScaled, zWidthScaled)
        GL11.glVertex3f(-xWidthScaled, -yWidthScaled, zWidthScaled)
        GL11.glVertex3f(-xWidthScaled, -yWidthScaled, -zWidthScaled)
        GL11.glEnd()
    }

    // Renders a plane from (-.5, 0, -.5) to (.5, 0, .5)
    private fun renderPlane() {
        GL11.glBegin(GL11.GL_QUADS)
        GL11.glColor3f(0.0f, 0.4f, 0.4f)
        GL11.glVertex3f(0.5f, 0.0f, 0.5f)
        GL11.glVertex3f(0.5f, 0.0f, -0.5f)
        GL11.glVertex3f(-0.5f, 0.0f, -0.5f)
        GL11.glVertex3f(-0.5f, 0.0f, 0.5f)
        GL11.glEnd()
    }

    private fun loop() {
        GL.createCapabilities()

        // Set the clear color
        GL11.glClearColor(0.6f, 0.7f, 0.8f, 1.0f)
        // Enable depth testing
        GL11.glEnable(GL11.GL_DEPTH_TEST)
        GL11.glEnable(GL11.GL_CULL_FACE)

        // Remember the current time.
        var lastTime = System.nanoTime()
        while (!GLFW.glfwWindowShouldClose(window)) {
            // Build time difference between this and first time.
            val thisTime = System.nanoTime()
            val diff = (thisTime - lastTime) / 1E9f

            // Run physics
            physicsWorld.simulate(1.0 / 60.0)

            Thread.sleep(1000 / 60)
            // Compute some rotation angle.

            // Make the viewport always fill the whole window.
            GL11.glViewport(0, 0, width, height)

            // Build the projection matrix. Watch out here for integer division
            // when computing the aspect ratio!
            projMatrix.setPerspective(Math.toRadians(40.0).toFloat(), width.toFloat() / height, 0.01f, 100.0f)
            GL11.glMatrixMode(GL11.GL_PROJECTION)
            GL11.glLoadMatrixf(projMatrix[floatBuffer])

            // Set lookat view matrix
            viewMatrix.setLookAt(
                cameraEyePos.x(), cameraEyePos.y(), cameraEyePos.z(),
                cameraLookAtPos.x(), cameraLookAtPos.y(), cameraLookAtPos.z(),
                0.0f, 1.0f, 0.0f
            )
            GL11.glMatrixMode(GL11.GL_MODELVIEW)
            GL11.glClear(GL11.GL_COLOR_BUFFER_BIT or GL11.GL_DEPTH_BUFFER_BIT)

            // Load the view matrix
            GL11.glLoadMatrixf(viewMatrix.get4x4(floatBuffer))

            // Render cubes
            for (body in physicsWorld.bodies) {
                GL11.glPushMatrix()
                GL11.glTranslatef(body.pose.p.x().toFloat(), body.pose.p.y().toFloat(), body.pose.p.z().toFloat())
                val axisAngle = AxisAngle4d().set(body.pose.q)
                GL11.glRotatef(
                    Math.toDegrees(axisAngle.angle).toFloat(), axisAngle.x.toFloat(), axisAngle.y.toFloat(),
                    axisAngle.z.toFloat()
                )

                // Render the shapes
                when (val bodyShape = body.shape) {
                    is TSDFVoxelShape -> {
                        GL11.glPushMatrix()
                        GL11.glScaled(bodyShape.scalingFactor, bodyShape.scalingFactor, bodyShape.scalingFactor)

                        bodyShape.layeredTSDF.forEachVoxel { posX, posY, posZ ->
                            GL11.glPushMatrix()
                            GL11.glTranslatef(
                                posX.toFloat() + bodyShape.voxelOffset.x().toFloat(),
                                posY.toFloat() + bodyShape.voxelOffset.y().toFloat(),
                                posZ.toFloat() + bodyShape.voxelOffset.z().toFloat()
                            )
                            renderCube()
                            GL11.glPopMatrix()
                        }
                        GL11.glPopMatrix()
                    }
                    is SphereShape -> {
                        GL11.glPushMatrix()
                        renderCube()
                        GL11.glPopMatrix()
                    }
                    is BoxShape -> {
                        GL11.glPushMatrix()
                        GL11.glScaled(bodyShape.xRadius / .5, bodyShape.yRadius / .5, bodyShape.zRadius / .5)
                        renderCube()
                        GL11.glPopMatrix()
                    }
                }
                GL11.glPopMatrix()
            }

            if (renderCenterOfBodies) {
                GL11.glDisable(GL11.GL_DEPTH_TEST)
                GL11.glEnable(GL11.GL_BLEND)
                GL11.glBlendFunc(GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA)
                for (body in physicsWorld.bodies) {
                    GL11.glPushMatrix()
                    GL11.glTranslatef(body.pose.p.x().toFloat(), body.pose.p.y().toFloat(), body.pose.p.z().toFloat())
                    val axisAngle = AxisAngle4d().set(body.pose.q)
                    GL11.glRotatef(
                        Math.toDegrees(axisAngle.angle).toFloat(), axisAngle.x.toFloat(), axisAngle.y.toFloat(),
                        axisAngle.z.toFloat()
                    )

                    renderCube(xWidth = .2f, yWidth = .2f, zWidth = .2f, color = Vector4f(1.0f, .412f, .706f, .3f))
                    GL11.glPopMatrix()
                }
                GL11.glDisable(GL11.GL_BLEND)
                GL11.glEnable(GL11.GL_DEPTH_TEST)
            }

            GL11.glPopMatrix()
            GLFW.glfwSwapBuffers(window)
            GLFW.glfwPollEvents()

            lastTime = thisTime
        }
    }
}
