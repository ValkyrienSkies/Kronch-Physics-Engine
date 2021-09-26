package org.valkyrienskies.krunch.constraints

import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.Joint
import org.valkyrienskies.krunch.applyBodyPairCorrectionDeltaLambdaOnlyWithRespectToNormal

class JointAttachmentConstraint(private val joint: Joint) : PositionConstraint {

    override var lambda: Double = 0.0
    override var prevLambda: Double = 0.0
    private val normal = Vector3d()

    override fun iterate(dt: Double, weight: Double) {
        if (normal.lengthSquared() == 0.0) return

        computeGlobalPositions { pos0: Vector3dc, pos1: Vector3dc ->
            val corr = Vector3d(pos1).sub(pos0)
            val deltaLambda =
                applyBodyPairCorrectionDeltaLambdaOnlyWithRespectToNormal(
                    joint.body0, joint.body1, corr, normal, joint.compliance, dt,
                    pos0, pos1, lambda
                )
            prevLambda = lambda
            lambda += weight * deltaLambda
        }
    }

    override fun reset() {
        lambda = 0.0
        prevLambda = 0.0

        computeGlobalPositions { pos0: Vector3dc, pos1: Vector3dc ->
            normal.set(pos1).sub(pos0)
            if (normal.lengthSquared() > 1e-24) {
                normal.normalize()
            } else {
                normal.zero()
            }
        }
    }

    private inline fun computeGlobalPositions(function: (pos0: Vector3dc, pos1: Vector3dc) -> Unit) {
        val pos0 = Vector3d(joint.localPose0.p)
        joint.body0?.pose?.transform(pos0)

        val pos1 = Vector3d(joint.localPose1.p)
        joint.body1?.pose?.transform(pos1)
        function(pos0, pos1)
    }

    override fun computeUpdateImpulses(
        force: Double,
        function: (body: Body, bodyLinearImpulse: Vector3dc?, bodyAngularImpulse: Vector3dc?) -> Unit
    ) {
        if (normal.lengthSquared() == 0.0) return

        computeGlobalPositions { body0PointPosInGlobal: Vector3dc, body1PointPosInGlobal: Vector3dc ->
            // Use deltaLambda when computing the impulse, since prevLambda has already been added to the bodies.
            val corr = Vector3d(normal).mul(force)

            joint.body0?.getCorrectionImpulses(corr, body0PointPosInGlobal) { linearImpulse, angularImpulse ->
                function(joint.body0, linearImpulse, angularImpulse)
            }

            corr.mul(-1.0)

            joint.body1?.getCorrectionImpulses(corr, body1PointPosInGlobal) { linearImpulse, angularImpulse ->
                function(joint.body1, linearImpulse, angularImpulse)
            }
        }
    }

    override fun forEachBody(function: (body: Body) -> Unit) {
        joint.body0?.let { function(it) }
        joint.body1?.let { function(it) }
    }
}
