package org.valkyrienskies.kronch

import org.joml.Vector3d

open class PhysicsWorld {
    val bodies: MutableList<Body> = ArrayList()
    val joints: MutableList<Joint> = ArrayList()

    open fun simulate(timeStep: Double) {
        val gravity = Vector3d(0.0, -10.0, 0.0)
        val numSubsteps = 40
        simulate(bodies, joints, timeStep, numSubsteps, gravity)
    }
}
