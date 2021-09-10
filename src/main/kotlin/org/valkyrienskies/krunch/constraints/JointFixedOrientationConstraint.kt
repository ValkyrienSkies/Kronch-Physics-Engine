package org.valkyrienskies.krunch.constraints

import org.joml.Quaterniond
import org.joml.Quaterniondc
import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.Joint
import org.valkyrienskies.krunch.applyBodyPairCorrectionDeltaLambdaOnlyWithRespectToNormal

class JointFixedOrientationConstraint(private val joint: Joint) : PositionConstraint {

    private var lambda: Double = 0.0
    private var prevLambda: Double = 0.0
    private val normal = Vector3d()

    override fun iterate(dt: Double, weight: Double) {
        if (normal.lengthSquared() == 0.0) return

        val omega = computeOmega()

        // Try to set [dotProduct] to 0
        val dotProduct = omega.dot(normal)

        // Force body0.pose.q and body1.pose.q to have an angle of 0 relative to omega
        val corr = Vector3d(normal).mul(dotProduct)

        val deltaLambda = applyBodyPairCorrectionDeltaLambdaOnlyWithRespectToNormal(
            body0 = joint.body0, body1 = joint.body1, corr = corr, constraintNormal = normal,
            compliance = joint.compliance, dt = dt, prevLambda = lambda
        )

        prevLambda = lambda
        lambda += weight * deltaLambda
    }

    override fun reset() {
        lambda = 0.0
        prevLambda = 0.0

        val omega = computeOmega()

        normal.set(omega)

        if (normal.lengthSquared() > 1e-24) {
            normal.normalize()
        } else {
            normal.zero()
        }
    }

    private fun computeOmega(): Vector3dc {
        val body0Rotation: Quaterniondc = joint.body0?.pose?.q ?: Quaterniond()
        val body1Rotation: Quaterniondc = joint.body1?.pose?.q ?: Quaterniond()

        val q = Quaterniond(body0Rotation)
        q.conjugate()
        q.premul(body1Rotation)
        val omega = Vector3d()
        omega.set(2.0 * q.x, 2.0 * q.y, 2.0 * q.z)
        return omega
    }

    override fun computeUpdateImpulses(
        function: (body: Body, bodyLinearImpulse: Vector3dc?, bodyAngularImpulse: Vector3dc?) -> Unit
    ) {
        if (normal.lengthSquared() == 0.0) return

        val body0PointPosInGlobal = null
        val body1PointPosInGlobal = null

        // Use deltaLambda when computing the impulse, since prevLambda has already been added to the bodies.
        val deltaLambda = lambda - prevLambda

        val corr = normal.mul(-deltaLambda, Vector3d())

        joint.body0?.getCorrectionImpulses(corr, body0PointPosInGlobal) { linearImpulse, angularImpulse ->
            function(joint.body0, linearImpulse, angularImpulse)
        }

        corr.mul(-1.0)

        joint.body1?.getCorrectionImpulses(corr, body1PointPosInGlobal) { linearImpulse, angularImpulse ->
            function(joint.body1, linearImpulse, angularImpulse)
        }
    }
}
