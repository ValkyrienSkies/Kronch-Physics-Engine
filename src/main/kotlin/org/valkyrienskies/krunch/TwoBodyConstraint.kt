package org.valkyrienskies.krunch

interface TwoBodyConstraint {

    fun iterate(dt: Double)

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
}
