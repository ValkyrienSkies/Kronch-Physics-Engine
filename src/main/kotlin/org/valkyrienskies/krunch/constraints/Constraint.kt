package org.valkyrienskies.krunch.constraints

import org.joml.Vector3dc
import org.valkyrienskies.krunch.Body

interface Constraint {

    fun iterate(dt: Double, weight: Double = 1.0)

    /**
     * Resets this constraint to its initial state. Used between sub-steps
     */
    fun reset()

    /**
     * Returns the impulses to be applied to each body involved in this constraint.
     */
    fun computeUpdateImpulses(
        force: Double,
        function: (
            body: Body, bodyLinearImpulse: Vector3dc?, bodyAngularImpulse: Vector3dc?
        ) -> Unit
    )

    /**
     * Compute the impulses applied from the change between [lambda] and [prevLambda]
     */
    fun computeDeltaImpulses(
        function: (
            body: Body, bodyLinearImpulse: Vector3dc?, bodyAngularImpulse: Vector3dc?
        ) -> Unit
    ) = computeUpdateImpulses(lambda - prevLambda, function)

    /**
     * Compute the total impulse applied by [lambda]
     */
    fun computeTotalImpulses(
        function: (
            body: Body, bodyLinearImpulse: Vector3dc?, bodyAngularImpulse: Vector3dc?
        ) -> Unit
    ) = computeUpdateImpulses(lambda, function)

    fun forEachBody(function: (body: Body) -> Unit)

    val lambda: Double
    val prevLambda: Double
}
