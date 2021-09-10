package org.valkyrienskies.krunch.constraints

import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.applyBodyPairCorrectionDeltaLambdaOnlyWithRespectToNormal
import kotlin.math.abs
import kotlin.math.min

class RestitutionConstraint(
    private val body0: Body,
    private val body0ContactPosInBody0Coordinates: Vector3dc,
    private val body1: Body,
    private val body1ContactPosInBody1Coordinates: Vector3dc,
    private val contactNormalInGlobalCoordinates: Vector3dc,
    private val restitutionCompliance: Double,
    private val collisionConstraint: CollisionConstraint
) : VelocityConstraint {

    private var lambda: Double = 0.0
    private var prevLambda: Double = 0.0

    override fun computeUpdateImpulses(
        function: (
            body: Body, bodyLinearImpulse: Vector3dc?, bodyAngularImpulse: Vector3dc?
        ) -> Unit
    ) {
        val body0PointPosInGlobal = body0.pose.transform(Vector3d(body0ContactPosInBody0Coordinates))
        val body1PointPosInGlobal = body1.pose.transform(Vector3d(body1ContactPosInBody1Coordinates))

        // Use deltaLambda when computing the impulse, since prevLambda has already been added to the bodies.
        val deltaLambda = lambda - prevLambda

        val corr = contactNormalInGlobalCoordinates.mul(-deltaLambda, Vector3d())

        body0.getCorrectionImpulses(corr, body0PointPosInGlobal) { linearImpulse, angularImpulse ->
            function(body0, linearImpulse, angularImpulse)
        }

        corr.mul(-1.0)

        body1.getCorrectionImpulses(corr, body1PointPosInGlobal) { linearImpulse, angularImpulse ->
            function(body1, linearImpulse, angularImpulse)
        }
    }

    override fun iterate(dt: Double, weight: Double) {
        // Don't correct restitution if the collision constraint didn't do anything
        if (collisionConstraint.getForceBetweenContacts() == 0.0) {
            prevLambda = lambda
            return
        }

        val positionInFirstBody = body0ContactPosInBody0Coordinates
        val positionInSecondBody = body1ContactPosInBody1Coordinates
        val normalThisSubStep = contactNormalInGlobalCoordinates
        val compliance = restitutionCompliance

        // For each collision contact, set the relative velocity of the collision points on both bodies to 0
        val body0CollisionPosInGlobal = body0.pose.transform(Vector3d(positionInFirstBody))
        val body1CollisionPosInGlobal = body1.pose.transform(Vector3d(positionInSecondBody))

        // Compute the current velocity along normal
        val body0VelocityAtPoint = body0.getVelocityAt(body0CollisionPosInGlobal)
        val body1VelocityAtPoint = body1.getVelocityAt(body1CollisionPosInGlobal)

        val relativeVelocity = body0VelocityAtPoint.sub(body1VelocityAtPoint, Vector3d())
        val relativeVelocityAlongNormal = normalThisSubStep.dot(relativeVelocity) // v_n

        // Compute the previous velocity along normal
        val relativeVelocityAlongNormalPrev =
            if (abs(relativeVelocityAlongNormal) > 2 * 10.0 * dt) {
                val body0VelocityAtPointPrev = body0.getPrevVelocityAt(body0CollisionPosInGlobal)
                val body1VelocityAtPointPrev = body1.getPrevVelocityAt(body1CollisionPosInGlobal)
                val relativeVelocityPrev =
                    body0VelocityAtPointPrev.sub(body1VelocityAtPointPrev, Vector3d())
                normalThisSubStep.dot(relativeVelocityPrev)
            } else {
                0.0
            }

        // Take the average of the coefficients of restitution of both bodies
        val coefficientOfRestitution = (body0.coefficientOfRestitution + body1.coefficientOfRestitution) / 2.0

        // [deltaVelocity] serves 2 purposes:
        // The first is to remove velocity added by [collision]
        // The second is to apply the coefficient of restitution to [collision]
        val deltaVelocity = normalThisSubStep.mul(
            -relativeVelocityAlongNormal +
                min(-coefficientOfRestitution * relativeVelocityAlongNormalPrev, 0.0),
            Vector3d()
        )

        val deltaLambda = applyBodyPairCorrectionDeltaLambdaOnlyWithRespectToNormal(
            body0, body1, deltaVelocity, normalThisSubStep, compliance, dt, body0CollisionPosInGlobal,
            body1CollisionPosInGlobal, lambda
        )

        prevLambda = lambda
        lambda += weight * deltaLambda
    }

    override fun reset() {
        prevLambda = 0.0
        lambda = 0.0
    }
}
