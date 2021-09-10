package org.valkyrienskies.krunch.solver

import org.valkyrienskies.krunch.constraints.Constraint

interface Solver {
    fun solvePositionConstraints(constraints: List<Constraint>, iterations: Int, dt: Double)
    fun solveVelocityConstraints(constraints: List<Constraint>, iterations: Int, dt: Double)
}
