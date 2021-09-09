package org.valkyrienskies.krunch.solver

import org.joml.Vector3d
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.TwoBodyConstraint

class JacobiSolver : Solver {

    override fun solvePositionConstraints(constraints: List<TwoBodyConstraint>, iterations: Int, dt: Double) {
        for (i in 1..iterations) {
            constraints.forEach {
                it.iterate(dt)
            }
            val linearImpulsesToAddMap = HashMap<Body, Vector3d>()
            val angularImpulsesToAddMap = HashMap<Body, Vector3d>()
            constraints.forEach {
                it.computeUpdateImpulses { body0, body0LinearImpulse, body0AngularImpulse,
                    body1, body1LinearImpulse, body1AngularImpulse ->
                    // For now, update immediately
                    if (!body0.isStatic) {
                        if (body0LinearImpulse != null)
                            linearImpulsesToAddMap.getOrPut(body0) { Vector3d() }.add(body0LinearImpulse)
                        if (body0AngularImpulse != null)
                            angularImpulsesToAddMap.getOrPut(body0) { Vector3d() }.add(body0AngularImpulse)
                    }
                    if (!body1.isStatic) {
                        if (body1LinearImpulse != null)
                            linearImpulsesToAddMap.getOrPut(body1) { Vector3d() }.add(body1LinearImpulse)
                        if (body1AngularImpulse != null)
                            angularImpulsesToAddMap.getOrPut(body1) { Vector3d() }.add(body1AngularImpulse)
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

    override fun solveVelocityConstraints(constraints: List<TwoBodyConstraint>, iterations: Int, dt: Double) {
        for (i in 1..iterations) {
            constraints.forEach {
                it.iterate(dt)
            }
            val linearImpulsesToAddMap = HashMap<Body, Vector3d>()
            val angularImpulsesToAddMap = HashMap<Body, Vector3d>()
            constraints.forEach {
                it.computeUpdateImpulses { body0, body0LinearImpulse, body0AngularImpulse,
                    body1, body1LinearImpulse, body1AngularImpulse ->
                    // For now, update immediately
                    if (!body0.isStatic) {
                        if (body0LinearImpulse != null)
                            linearImpulsesToAddMap.getOrPut(body0) { Vector3d() }.add(body0LinearImpulse)
                        if (body0AngularImpulse != null)
                            angularImpulsesToAddMap.getOrPut(body0) { Vector3d() }.add(body0AngularImpulse)
                    }
                    if (!body1.isStatic) {
                        if (body1LinearImpulse != null)
                            linearImpulsesToAddMap.getOrPut(body1) { Vector3d() }.add(body1LinearImpulse)
                        if (body1AngularImpulse != null)
                            angularImpulsesToAddMap.getOrPut(body1) { Vector3d() }.add(body1AngularImpulse)
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
