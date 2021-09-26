package org.valkyrienskies.krunch.constraints

import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.applyBodyPairCorrectionDeltaLambdaOnlyWithRespectToNormal

abstract class RotationPositionConstraint(
    private val body0: Body?, private val body1: Body?, private val compliance: Double
) : PositionConstraint {
    override var lambda: Double = 0.0
    override var prevLambda: Double = 0.0
    private val normal = Vector3d()
    private var needsNormal = true

    override fun iterate(dt: Double, weight: Double) {
        if (normal.lengthSquared() == 0.0 && !needsNormal) {
            prevLambda = lambda
            return
        }

        val omega = computeRotationCorrection()

        if (needsNormal) {
            needsNormal = false
            if (omega.lengthSquared() < 1e-24) {
                prevLambda = lambda
                return
            }
            normal.set(omega).normalize()
        }

        // Try to set [dotProduct] to 0
        val dotProduct = omega.dot(normal)

        // Force body0.pose.q and body1.pose.q to have an angle of 0 relative to omega
        val corr = Vector3d(normal).mul(dotProduct)

        val deltaLambda = applyBodyPairCorrectionDeltaLambdaOnlyWithRespectToNormal(
            body0 = body0, body1 = body1, corr = corr, constraintNormal = normal,
            compliance = compliance, dt = dt, prevLambda = lambda
        )

        prevLambda = lambda
        lambda += weight * deltaLambda
    }

    override fun reset() {
        lambda = 0.0
        prevLambda = 0.0

        normal.zero()
        needsNormal = true
    }

    abstract fun computeRotationCorrection(): Vector3dc

    override fun computeUpdateImpulses(
        function: (body: Body, bodyLinearImpulse: Vector3dc?, bodyAngularImpulse: Vector3dc?) -> Unit
    ) {
        if (normal.lengthSquared() == 0.0) return

        val body0PointPosInGlobal = null
        val body1PointPosInGlobal = null

        // Use deltaLambda when computing the impulse, since prevLambda has already been added to the bodies.
        val deltaLambda = lambda - prevLambda

        val corr = normal.mul(-deltaLambda, Vector3d())

        body0?.getCorrectionImpulses(corr, body0PointPosInGlobal) { linearImpulse, angularImpulse ->
            function(body0, linearImpulse, angularImpulse)
        }

        corr.mul(-1.0)

        body1?.getCorrectionImpulses(corr, body1PointPosInGlobal) { linearImpulse, angularImpulse ->
            function(body1, linearImpulse, angularImpulse)
        }
    }

    override fun forEachBody(function: (body: Body) -> Unit) {
        body0?.let { function(it) }
        body1?.let { function(it) }
    }
}
