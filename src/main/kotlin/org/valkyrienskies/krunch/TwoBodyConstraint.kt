package org.valkyrienskies.krunch

import org.joml.Vector3dc

interface TwoBodyConstraint {

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

    fun computeUpdateImpulses(
        function: (
            body0: Body, body0LinearImpulse: Vector3dc?, body0AngularImpulse: Vector3dc?,
            body1: Body, body1LinearImpulse: Vector3dc?, body1AngularImpulse: Vector3dc?
        ) -> Unit
    )
}
