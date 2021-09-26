package org.valkyrienskies.krunch.solver

import org.valkyrienskies.krunch.constraints.Constraint

class GaussSeidelSolver : Solver {

    // See https://en.wikipedia.org/wiki/Jacobi_method#Weighted_Jacobi_method
    private val positionConstraintWeight = 1.0
    private val velocityConstraintWeight = 1.0

    override fun solvePositionConstraints(constraints: List<Constraint>, iterations: Int, dt: Double) {
        for (i in 1..iterations) {
            constraints.forEach {
                it.iterate(dt, positionConstraintWeight)

                // Update immediately
                it.computeDeltaImpulses { body, bodyLinearImpulse, bodyAngularImpulse ->
                    // For now, update immediately
                    if (!body.isStatic) {
                        if (bodyLinearImpulse != null) body.pose.p.add(bodyLinearImpulse)
                        if (bodyAngularImpulse != null) body.applyRotation(bodyAngularImpulse)
                    }
                }
            }
        }
    }

    override fun solveVelocityConstraints(constraints: List<Constraint>, iterations: Int, dt: Double) {
        for (i in 1..iterations) {
            constraints.forEach {
                it.iterate(dt, velocityConstraintWeight)

                // Update immediately
                it.computeDeltaImpulses { body, bodyLinearImpulse, bodyAngularImpulse ->
                    // For now, update immediately
                    if (!body.isStatic) {
                        if (bodyLinearImpulse != null) body.vel.add(bodyLinearImpulse)
                        if (bodyAngularImpulse != null) body.omega.add(bodyAngularImpulse)
                    }
                }
            }
        }
    }
}
