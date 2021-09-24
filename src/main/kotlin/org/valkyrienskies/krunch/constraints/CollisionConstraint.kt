package org.valkyrienskies.krunch.constraints

import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.applyBodyPairCorrectionDeltaLambdaOnlyWithRespectToNormal
import kotlin.math.abs
import kotlin.math.max

class CollisionConstraint(
    internal val body0: Body,
    internal val body0ContactPosInBody0Coordinates: Vector3dc,
    internal val body1: Body,
    internal val body1ContactPosInBody1Coordinates: Vector3dc,
    internal val contactNormalInGlobalCoordinates: Vector3dc,
    private val collisionCompliance: Double
) : PositionConstraint {

    internal var lambda: Double = 0.0
    internal var prevLambda: Double = 0.0

    override fun computeUpdateImpulses(
        function: (
            body: Body, bodyLinearImpulse: Vector3dc?, bodyAngularImpulse: Vector3dc?
        ) -> Unit
    ) {
        val body0PointPosInGlobal = body0.pose.transform(Vector3d(body0ContactPosInBody0Coordinates))
        val body1PointPosInGlobal = body1.pose.transform(Vector3d(body1ContactPosInBody1Coordinates))

        // Use deltaLambda when computing the impulse, since prevLambda has already been added to the bodies.
        val deltaLambda = lambda - prevLambda

        if (abs(deltaLambda) < 1e-12) return // Skip

        val corr = contactNormalInGlobalCoordinates.mul(-deltaLambda, Vector3d())

        body0.getCorrectionImpulses(corr, body0PointPosInGlobal) { linearImpulse, angularImpulse ->
            function(body0, linearImpulse, angularImpulse)
        }

        corr.mul(-1.0)

        body1.getCorrectionImpulses(corr, body1PointPosInGlobal) { linearImpulse, angularImpulse ->
            function(body1, linearImpulse, angularImpulse)
        }
    }

    override fun forEachBody(function: (body: Body) -> Unit) {
        function(body0)
        function(body1)
    }

    override fun iterate(dt: Double, weight: Double) {
        // Update lambda
        val body0PointPosInGlobal = body0.pose.transform(Vector3d(body0ContactPosInBody0Coordinates))
        val body1PointPosInGlobal = body1.pose.transform(Vector3d(body1ContactPosInBody1Coordinates))

        val positionDifference = body0PointPosInGlobal.sub(body1PointPosInGlobal, Vector3d())
        val d = -contactNormalInGlobalCoordinates.dot(positionDifference)

        // This part doesn't make sense to me now, but it fixes a lot of problems to make [corr] negative
        val corr = contactNormalInGlobalCoordinates.mul(d, Vector3d())

        val deltaLambda = applyBodyPairCorrectionDeltaLambdaOnlyWithRespectToNormal(
            body0, body1, corr, contactNormalInGlobalCoordinates, collisionCompliance, dt, body0PointPosInGlobal,
            body1PointPosInGlobal, lambda
        )

        prevLambda = lambda
        lambda += weight * deltaLambda
        // Don't let lambda go below 0 (Below 0 would move the bodies deeper into each-other instead of further away)
        lambda = max(lambda, 0.0)
    }

    override fun reset() {
        prevLambda = 0.0
        lambda = 0.0
    }

    fun getContactNormalInGlobalCoordinates() = contactNormalInGlobalCoordinates

    fun getForceBetweenContacts() = lambda
}
