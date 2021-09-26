package org.valkyrienskies.krunch.solver

import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.Posec
import org.valkyrienskies.krunch.constraints.Constraint

class JacobiSolver : Solver {

    companion object {
        private const val WEIGHT = .3
    }

    override fun solvePositionConstraints(
        bodies: List<Body>, constraints: List<Constraint>, iterations: Int, dt: Double
    ) {
        val bodyInitialPoses: MutableMap<Body, Posec> = HashMap()
        bodies.forEach { body ->
            bodyInitialPoses[body] = body.pose.clone()
        }
        for (i in 1..iterations) {
            val weight = WEIGHT
            constraints.forEach {
                it.iterate(dt, weight)
            }
            val linearImpulsesToAddMap = HashMap<Body, Vector3d>()
            val angularImpulsesToAddMap = HashMap<Body, Vector3d>()
            constraints.forEach {
                it.computeTotalImpulses { body, bodyLinearImpulse, bodyAngularImpulse ->
                    // For now, update immediately
                    if (!body.isStatic) {
                        if (bodyLinearImpulse != null)
                            linearImpulsesToAddMap.getOrPut(body) { Vector3d() }.add(bodyLinearImpulse)
                        if (bodyAngularImpulse != null)
                            angularImpulsesToAddMap.getOrPut(body) { Vector3d() }.add(bodyAngularImpulse)
                    }
                }
            }

            bodyInitialPoses.forEach { (body, originalPose) ->
                body.pose.set(originalPose)
            }

            linearImpulsesToAddMap.forEach { (body, linearImpulse) ->
                body.pose.p.add(linearImpulse)
            }

            angularImpulsesToAddMap.forEach { (body, angularImpulse) ->
                body.applyRotation(angularImpulse)
            }
        }
    }

    override fun solveVelocityConstraints(
        bodies: List<Body>, constraints: List<Constraint>, iterations: Int, dt: Double
    ) {
        val bodyInitialVelocities: MutableMap<Body, Pair<Vector3dc, Vector3dc>> = HashMap()
        bodies.forEach { body ->
            bodyInitialVelocities[body] = Pair(Vector3d(body.vel), Vector3d(body.omega))
        }

        for (i in 1..iterations) {
            val weight = WEIGHT
            constraints.forEach {
                it.iterate(dt, weight)
            }
            val linearImpulsesToAddMap = HashMap<Body, Vector3d>()
            val angularImpulsesToAddMap = HashMap<Body, Vector3d>()
            constraints.forEach {
                it.computeTotalImpulses { body, bodyLinearImpulse, bodyAngularImpulse ->
                    // For now, update immediately
                    if (!body.isStatic) {
                        if (bodyLinearImpulse != null)
                            linearImpulsesToAddMap.getOrPut(body) { Vector3d() }.add(bodyLinearImpulse)
                        if (bodyAngularImpulse != null)
                            angularImpulsesToAddMap.getOrPut(body) { Vector3d() }.add(bodyAngularImpulse)
                    }
                }
            }

            bodyInitialVelocities.forEach { (body, initialVelocityPair) ->
                body.vel.set(initialVelocityPair.first)
                body.omega.set(initialVelocityPair.second)
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
