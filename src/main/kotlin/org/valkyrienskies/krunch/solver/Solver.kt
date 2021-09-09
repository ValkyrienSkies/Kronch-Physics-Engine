package org.valkyrienskies.krunch.solver

import org.valkyrienskies.krunch.TwoBodyConstraint

interface Solver {
    fun solvePositionConstraints(constraints: List<TwoBodyConstraint>, iterations: Int, dt: Double)
    fun solveVelocityConstraints(constraints: List<TwoBodyConstraint>, iterations: Int, dt: Double)
}
