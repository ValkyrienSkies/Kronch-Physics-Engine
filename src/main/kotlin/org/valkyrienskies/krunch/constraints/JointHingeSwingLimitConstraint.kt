package org.valkyrienskies.krunch.constraints

import org.joml.Vector3dc
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.Joint

class JointHingeSwingLimitConstraint(private val joint: Joint) : PositionConstraint {

    private var lambda: Double = 0.0
    private var prevLambda: Double = 0.0

    override fun iterate(dt: Double, weight: Double) {
        TODO("Not yet implemented")
    }

    override fun reset() {
        lambda = 0.0
        prevLambda = 0.0
    }

    override fun computeUpdateImpulses(
        function: (body: Body, bodyLinearImpulse: Vector3dc?, bodyAngularImpulse: Vector3dc?) -> Unit
    ) {
        TODO("Not yet implemented")
    }
}
