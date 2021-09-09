package org.valkyrienskies.krunch.solver

import org.valkyrienskies.krunch.TwoBodyConstraint

class GaussSeidelSolver : Solver {

    // See https://en.wikipedia.org/wiki/Jacobi_method#Weighted_Jacobi_method
    private val positionConstraintWeight = .25
    private val velocityConstraintWeight = .25

    override fun solvePositionConstraints(constraints: List<TwoBodyConstraint>, iterations: Int, dt: Double) {
        for (i in 1..iterations) {
            constraints.forEach {
                it.iterate(dt, positionConstraintWeight)

                // Update immediately
                it.computeUpdateImpulses { body0, body0LinearImpulse, body0AngularImpulse,
                    body1, body1LinearImpulse, body1AngularImpulse ->
                    // For now, update immediately
                    if (!body0.isStatic) {
                        if (body0LinearImpulse != null) body0.pose.p.add(body0LinearImpulse)
                        if (body0AngularImpulse != null) body0.applyRotation(body0AngularImpulse)
                    }
                    if (!body1.isStatic) {
                        if (body1LinearImpulse != null) body1.pose.p.add(body1LinearImpulse)
                        if (body1AngularImpulse != null) body1.applyRotation(body1AngularImpulse)
                    }
                }
            }
        }
    }

    override fun solveVelocityConstraints(constraints: List<TwoBodyConstraint>, iterations: Int, dt: Double) {
        for (i in 1..iterations) {
            constraints.forEach {
                it.iterate(dt, velocityConstraintWeight)

                // Update immediately
                it.computeUpdateImpulses { body0, body0LinearImpulse, body0AngularImpulse,
                    body1, body1LinearImpulse, body1AngularImpulse ->
                    // For now, update immediately
                    if (!body0.isStatic) {
                        if (body0LinearImpulse != null) body0.vel.add(body0LinearImpulse)
                        if (body0AngularImpulse != null) body0.omega.add(body0AngularImpulse)
                    }
                    if (!body1.isStatic) {
                        if (body1LinearImpulse != null) body1.vel.add(body1LinearImpulse)
                        if (body1AngularImpulse != null) body1.omega.add(body1AngularImpulse)
                    }
                }
            }
        }
    }
}
