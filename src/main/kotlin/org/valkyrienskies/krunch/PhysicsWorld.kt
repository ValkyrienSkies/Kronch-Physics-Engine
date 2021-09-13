package org.valkyrienskies.krunch

import org.joml.Vector3dc
import org.valkyrienskies.krunch.collision.broadphase.BroadphaseInterface
import org.valkyrienskies.krunch.collision.broadphase.DbvtBroadphase

open class PhysicsWorld {
    val bodies: MutableList<Body> = ArrayList()
    val joints: MutableList<Joint> = ArrayList()

    var settings = KrunchPhysicsWorldSettings()

    private val broadPhase: BroadphaseInterface = DbvtBroadphase()

    open fun simulate(gravity: Vector3dc, timeStep: Double) {
        simulate(bodies, joints, broadPhase, gravity, timeStep, settings)
    }
}
