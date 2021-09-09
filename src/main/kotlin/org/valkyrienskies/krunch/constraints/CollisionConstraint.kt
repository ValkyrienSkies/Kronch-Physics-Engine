package org.valkyrienskies.krunch.constraints

import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.PAIR_CORRECTION_MIN_LENGTH
import org.valkyrienskies.krunch.applyBodyPairCorrectionDeltaLambdaOnly
import kotlin.math.abs
import kotlin.math.min

class CollisionConstraint(
    internal val body0: Body,
    internal val body0ContactPosInBody0Coordinates: Vector3dc,
    internal val body1: Body,
    internal val body1ContactPosInBody1Coordinates: Vector3dc,
    internal val contactNormalInGlobalCoordinates: Vector3dc,
    private val collisionCompliance: Double
) : Constraint {

    internal var lambda: Double = 0.0
    internal var prevLambda: Double = 0.0

    override fun computeUpdateImpulses(
        function: (
            body: Body, bodyLinearImpulse: Vector3dc?, bodyAngularImpulse: Vector3dc?
        ) -> Unit
    ) {
        var body0LinearImpulse: Vector3dc? = null
        var body0AngularImpulse: Vector3dc? = null
        var body1LinearImpulse: Vector3dc? = null
        var body1AngularImpulse: Vector3dc? = null

        val body0PointPosInGlobal = body0.pose.transform(Vector3d(body0ContactPosInBody0Coordinates))
        val body1PointPosInGlobal = body1.pose.transform(Vector3d(body1ContactPosInBody1Coordinates))

        // Use deltaLambda when computing the impulse, since prevLambda has already been added to the bodies.
        val deltaLambda = lambda - prevLambda

        if (abs(deltaLambda) < 1e-12) return // Skip

        val corr = contactNormalInGlobalCoordinates.mul(deltaLambda, Vector3d())

        body0.getCorrectionImpulses(corr, body0PointPosInGlobal) { linearImpulse, angularImpulse ->
            body0LinearImpulse = linearImpulse
            body0AngularImpulse = angularImpulse
        }

        corr.mul(-1.0)

        body1.getCorrectionImpulses(corr, body1PointPosInGlobal) { linearImpulse, angularImpulse ->
            body1LinearImpulse = linearImpulse
            body1AngularImpulse = angularImpulse
        }

        function(body0, body0LinearImpulse, body0AngularImpulse)
        function(body1, body1LinearImpulse, body1AngularImpulse)
    }

    override fun iterate(dt: Double, weight: Double) {
        // Update lambda
        val body0PointPosInGlobal = body0.pose.transform(Vector3d(body0ContactPosInBody0Coordinates))
        val body1PointPosInGlobal = body1.pose.transform(Vector3d(body1ContactPosInBody1Coordinates))

        val positionDifference = body0PointPosInGlobal.sub(body1PointPosInGlobal, Vector3d())
        val d = contactNormalInGlobalCoordinates.dot(positionDifference)

        if (d < PAIR_CORRECTION_MIN_LENGTH) {
            // No longer colliding, skip this contact
            // There should be no update to lambda, so set prevLambda = lambda
            prevLambda = lambda
            return
        }

        // This part doesn't make sense to me now, but it fixes a lot of problems to make [corr] negative
        val corr = contactNormalInGlobalCoordinates.mul(-d, Vector3d())

        val deltaLambda = applyBodyPairCorrectionDeltaLambdaOnly(
            body0, body1, corr, collisionCompliance, dt, body0PointPosInGlobal, body1PointPosInGlobal, lambda
        )

        if (abs(deltaLambda) < 1e-12) return

        prevLambda = lambda
        lambda += weight * deltaLambda
        // Don't let lambda go above 0 (Above 0 would move the bodies deeper into each-other instead of further away)
        lambda = min(lambda, 0.0)
    }

    override fun reset() {
        prevLambda = 0.0
        lambda = 0.0
    }

    override fun shouldApplyThisSubStep(): Boolean {
        if (contactNormalInGlobalCoordinates.lengthSquared() < .99) throw IllegalStateException(
            "The collision normal $contactNormalInGlobalCoordinates is not normal!"
        )

        val body0PointPosInGlobal = body0.pose.transform(Vector3d(body0ContactPosInBody0Coordinates))
        val body1PointPosInGlobal = body1.pose.transform(Vector3d(body1ContactPosInBody1Coordinates))

        val positionDifference = body0PointPosInGlobal.sub(body1PointPosInGlobal, Vector3d())
        val d = contactNormalInGlobalCoordinates.dot(positionDifference)

        return d >= PAIR_CORRECTION_MIN_LENGTH
    }

    fun getContactNormalInGlobalCoordinates() = contactNormalInGlobalCoordinates

    fun getForceBetweenContacts() = lambda
}
