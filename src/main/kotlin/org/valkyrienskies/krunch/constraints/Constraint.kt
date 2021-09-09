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
     * Returns true iff this constraint should be applied this sub-step, false otherwise.
     *
     * For example, a collision constraint should only be applied for a sub-step if the penetration distance is > 0,
     * otherwise it should be ignored.
     */
    fun shouldApplyThisSubStep(): Boolean

    /**
     * Returns the impulses to be applied to each body involved in this constraint.
     */
    fun computeUpdateImpulses(
        function: (
            body: Body, bodyLinearImpulse: Vector3dc?, bodyAngularImpulse: Vector3dc?
        ) -> Unit
    )
}
