package org.valkyrienskies.krunch.solver

import org.joml.Vector3d
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.constraints.Constraint

class JacobiSolver : Solver {

    // See https://en.wikipedia.org/wiki/Jacobi_method#Weighted_Jacobi_method
    private val positionConstraintWeight = 1.0 / 8.0
    private val velocityConstraintWeight = 1.0 / 8.0

    override fun solvePositionConstraints(constraints: List<Constraint>, iterations: Int, dt: Double) {
        for (i in 1..iterations) {
            constraints.forEach {
                it.iterate(dt, positionConstraintWeight)
            }
            val linearImpulsesToAddMap = HashMap<Body, Vector3d>()
            val angularImpulsesToAddMap = HashMap<Body, Vector3d>()
            constraints.forEach {
                it.computeUpdateImpulses { body, bodyLinearImpulse, bodyAngularImpulse ->
                    // For now, update immediately
                    if (!body.isStatic) {
                        if (bodyLinearImpulse != null)
                            linearImpulsesToAddMap.getOrPut(body) { Vector3d() }.add(bodyLinearImpulse)
                        if (bodyAngularImpulse != null)
                            angularImpulsesToAddMap.getOrPut(body) { Vector3d() }.add(bodyAngularImpulse)
                    }
                }
            }

            linearImpulsesToAddMap.forEach { (body, linearImpulse) ->
                body.pose.p.add(linearImpulse)
            }

            angularImpulsesToAddMap.forEach { (body, angularImpulse) ->
                body.applyRotation(angularImpulse)
            }
        }
    }

    override fun solveVelocityConstraints(constraints: List<Constraint>, iterations: Int, dt: Double) {
        for (i in 1..iterations) {
            constraints.forEach {
                it.iterate(dt, velocityConstraintWeight)
            }
            val linearImpulsesToAddMap = HashMap<Body, Vector3d>()
            val angularImpulsesToAddMap = HashMap<Body, Vector3d>()
            constraints.forEach {
                it.computeUpdateImpulses { body, bodyLinearImpulse, bodyAngularImpulse ->
                    // For now, update immediately
                    if (!body.isStatic) {
                        if (bodyLinearImpulse != null)
                            linearImpulsesToAddMap.getOrPut(body) { Vector3d() }.add(bodyLinearImpulse)
                        if (bodyAngularImpulse != null)
                            angularImpulsesToAddMap.getOrPut(body) { Vector3d() }.add(bodyAngularImpulse)
                    }
                }
            }

            linearImpulsesToAddMap.forEach { (body, linearImpulse) ->
                body.vel.add(linearImpulse)
            }

            angularImpulsesToAddMap.forEach { (body, angularImpulse) ->
                body.omega.add(angularImpulse)
            }
        }
    }
}
