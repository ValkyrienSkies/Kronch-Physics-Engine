package org.valkyrienskies.krunch.solver

import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.constraints.Constraint

interface Solver {
    fun solvePositionConstraints(bodies: List<Body>, constraints: List<Constraint>, iterations: Int, dt: Double)
    fun solveVelocityConstraints(bodies: List<Body>, constraints: List<Constraint>, iterations: Int, dt: Double)
}
