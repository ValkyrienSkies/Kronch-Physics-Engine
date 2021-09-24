package org.valkyrienskies.krunch.solver

import org.joml.Vector3d
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.constraints.Constraint

class JacobiSolver : Solver {

    companion object {
        private const val WEIGHT = .3
    }

    override fun solvePositionConstraints(constraints: List<Constraint>, iterations: Int, dt: Double) {
        for (i in 1..iterations) {
            val weight = WEIGHT
            constraints.forEach {
                it.iterate(dt, weight)
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
            val weight = WEIGHT
            constraints.forEach {
                it.iterate(dt, weight)
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
