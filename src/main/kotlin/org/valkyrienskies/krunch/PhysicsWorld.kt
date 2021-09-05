package org.valkyrienskies.krunch

import org.joml.Vector3dc

open class PhysicsWorld {
    val bodies: MutableList<Body> = ArrayList()
    val joints: MutableList<Joint> = ArrayList()

    val settings = KrunchPhysicsWorldSettings()

    open fun simulate(gravity: Vector3dc, timeStep: Double) {
        simulate(bodies, joints, gravity, timeStep, settings)
    }
}
