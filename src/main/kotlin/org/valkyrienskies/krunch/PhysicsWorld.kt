package org.valkyrienskies.krunch

import org.joml.Vector3dc

open class PhysicsWorld {
    val bodies: MutableList<Body> = ArrayList()
    val joints: MutableList<Joint> = ArrayList()

    open fun simulate(gravity: Vector3dc, numSubSteps: Int, timeStep: Double) {
        simulate(bodies, joints, timeStep, numSubSteps, gravity)
    }
}
