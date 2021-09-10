package org.valkyrienskies.krunch

import org.joml.Quaterniond
import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.SolverType.GAUSS_SEIDEL
import org.valkyrienskies.krunch.SolverType.JACOBI
import org.valkyrienskies.krunch.collision.CollisionResult
import org.valkyrienskies.krunch.collision.colliders.ColliderResolver
import org.valkyrienskies.krunch.constraints.CollisionConstraint
import org.valkyrienskies.krunch.constraints.PositionConstraint
import org.valkyrienskies.krunch.constraints.RestitutionConstraint
import org.valkyrienskies.krunch.constraints.VelocityConstraint
import org.valkyrienskies.krunch.solver.GaussSeidelSolver
import org.valkyrienskies.krunch.solver.JacobiSolver
import org.valkyrienskies.krunch.solver.Solver
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.min

// pretty much one-for-one port of https://github.com/matthias-research/pages/blob/master/challenges/PBD.js

const val maxRotationPerSubstep = 0.5

internal const val PAIR_CORRECTION_MIN_LENGTH = 1e-10

fun applyBodyPairCorrectionDeltaLambdaOnly(
    body0: Body?, body1: Body?, corr: Vector3dc, compliance: Double,
    dt: Double, pos0: Vector3dc? = null, pos1: Vector3dc? = null,
    prevLambda: Double = 0.0, maxLambda: Double = Double.MAX_VALUE
): Double {
    val C = corr.length()
    if (C == 0.0)
        return 0.0

    val normal = Vector3d(corr)
    normal.normalize()

    val w0 = if (body0 != null && !body0.isStatic) body0.getInverseMass(normal, pos0) else 0.0
    val w1 = if (body1 != null && !body1.isStatic) body1.getInverseMass(normal, pos1) else 0.0

    val w = w0 + w1
    if (w == 0.0)
        return 0.0

    val deltaLambda = (-C - prevLambda * compliance) / (w + (compliance / (dt * dt)))

    if (abs(deltaLambda) > maxLambda) {
        // This part is only used for static friction (limit the strength of static friction)
        return 0.0
    }

    return deltaLambda
}

fun applyBodyPairCorrectionDeltaLambdaOnlyWithRespectToNormal(
    body0: Body?, body1: Body?, corr: Vector3dc, constraintNormal: Vector3dc, compliance: Double,
    dt: Double, pos0: Vector3dc? = null, pos1: Vector3dc? = null,
    prevLambda: Double = 0.0, maxLambda: Double = Double.MAX_VALUE
): Double {
    val C = corr.length()
    if (C == 0.0)
        return 0.0

    val normal = Vector3d(corr)
    normal.normalize()

    val w0 = if (body0 != null && !body0.isStatic) body0.getInverseMass(normal, pos0) else 0.0
    val w1 = if (body1 != null && !body1.isStatic) body1.getInverseMass(normal, pos1) else 0.0

    val w = w0 + w1
    if (w == 0.0)
        return 0.0

    val isCorrSameDirectionAsConstraintNormal = constraintNormal.dot(corr) > 0

    val prevLambdaWithRespectToCorr: Double = if (isCorrSameDirectionAsConstraintNormal) prevLambda else -prevLambda

    val deltaLambda = (-C - prevLambdaWithRespectToCorr * compliance) / (w + (compliance / (dt * dt)))

    if (abs(deltaLambda) > maxLambda) {
        // This part is only used for static friction (limit the strength of static friction)
        return 0.0
    }

    return if (isCorrSameDirectionAsConstraintNormal) deltaLambda else -deltaLambda
}

/**
 * Returns the lambda used in this correction computation
 */
fun applyBodyPairCorrection(
    body0: Body?, body1: Body?, corr: Vector3dc, compliance: Double,
    dt: Double, pos0: Vector3dc? = null, pos1: Vector3dc? = null, velocityLevel: Boolean = false,
    prevLambda: Double = 0.0, maxLambda: Double = Double.MAX_VALUE
): Double {
    val C = corr.length()
    if (C == 0.0)
        return prevLambda

    val normal = Vector3d(corr)
    normal.normalize()

    val w0 = if (body0 != null && !body0.isStatic) body0.getInverseMass(normal, pos0) else 0.0
    val w1 = if (body1 != null && !body1.isStatic) body1.getInverseMass(normal, pos1) else 0.0

    val w = w0 + w1
    if (w == 0.0)
        return prevLambda

    val deltaLambda = (-C - prevLambda * compliance) / (w + (compliance / (dt * dt)))

    if (abs(deltaLambda) > maxLambda) {
        // This part is only used for static friction (limit the strength of static friction)
        return prevLambda
    }

    val newLambda = prevLambda + deltaLambda

    normal.mul(-newLambda)

    if (body0 != null && !body0.isStatic) {
        body0.applyCorrection(normal, pos0, velocityLevel)
    }

    if (body1 != null && !body1.isStatic) {
        normal.mul(-1.0)
        body1.applyCorrection(normal, pos1, velocityLevel)
    }
    return newLambda
}

fun limitAngle(
    body0: Body?, body1: Body?, n: Vector3d, a: Vector3d, b: Vector3d, minAngle: Double, maxAngle: Double,
    compliance: Double, dt: Double, maxCorr: Double = Math.PI
) {
    // the key function to handle all angular joint limits
    val c = a.cross(b, Vector3d())

    var phi = asin(c.dot(n))
    if (a.dot(b) < 0.0)
        phi = Math.PI - phi

    if (phi > Math.PI)
        phi -= 2.0 * Math.PI
    if (phi < -Math.PI)
        phi += 2.0 * Math.PI

    if (phi < minAngle || phi > maxAngle) {
        phi = minAngle.coerceIn(phi, maxAngle)

        val q = Quaterniond()
        q.setAngleAxis(phi, n)

        val omega = Vector3d(a)
        omega.rotate(q)
        omega.cross(b)

        phi = omega.length()
        if (phi > maxCorr)
            omega.mul(maxCorr / phi)

        applyBodyPairCorrection(body0, body1, omega, compliance, dt)
    }
}

private fun createCollisionConstraints(
    collisionData: List<CollisionData>, settings: KrunchPhysicsWorldSettingsc
): List<CollisionConstraint> {
    val collisionConstraints = ArrayList<CollisionConstraint>()
    collisionData.forEach { data ->
        data.collisionResult.collisionPoints.forEach { collisionPair ->
            val collisionConstraint = CollisionConstraint(
                data.body0, collisionPair.positionInFirstBody, data.body1, collisionPair.positionInSecondBody,
                collisionPair.originalCollisionNormal, settings.collisionCompliance
            )
            collisionConstraints.add(collisionConstraint)
        }
    }
    return collisionConstraints
}

private fun createRestitutionConstraints(
    collisionConstraints: List<CollisionConstraint>, settings: KrunchPhysicsWorldSettingsc
): List<RestitutionConstraint> {
    val restitutionConstraints = ArrayList<RestitutionConstraint>()
    collisionConstraints.forEach { collisionConstraint ->
        val restitutionConstraint = RestitutionConstraint(
            collisionConstraint.body0, collisionConstraint.body0ContactPosInBody0Coordinates, collisionConstraint.body1,
            collisionConstraint.body1ContactPosInBody1Coordinates,
            collisionConstraint.contactNormalInGlobalCoordinates, settings.collisionRestitutionCompliance,
            collisionConstraint
        )
        restitutionConstraints.add(restitutionConstraint)
    }
    return restitutionConstraints
}

fun simulate(
    bodies: List<Body>,
    joints: List<Joint>,
    gravity: Vector3dc,
    timeStep: Double,
    settings: KrunchPhysicsWorldSettingsc
) {
    val solver: Solver = when (settings.solverType) {
        GAUSS_SEIDEL -> GaussSeidelSolver()
        JACOBI -> JacobiSolver()
    }

    val dt = timeStep / settings.subSteps

    // Only solve collision detection once per time step
    val collisions = generateCollisions(bodies, timeStep, speculativeContactDistance = 0.05)
    val collisionConstraints = createCollisionConstraints(collisions, settings)
    val jointPositionConstraints = ArrayList<PositionConstraint>()
    joints.forEach {
        jointPositionConstraints.addAll(it.getPositionConstraints())
    }

    val restitutionConstraints = createRestitutionConstraints(collisionConstraints, settings)

    val positionConstraints: List<PositionConstraint> = collisionConstraints + jointPositionConstraints
    val velocityConstraints: List<VelocityConstraint> = restitutionConstraints

    for (step in 0 until settings.subSteps) {
        // Step 1, integrate velocity into position
        for (body in bodies)
            if (!body.isStatic) body.integrate(dt, gravity)

        // Step 2, solve positional constraints (like joints and contacts)
        positionConstraints.forEach {
            it.reset()
        }
        solver.solvePositionConstraints(positionConstraints, settings.iterations, dt)

        // Step 2.5, add static friction using a Gauss-Seidel, regardless of [solver].
        // (This gives us unconditional stability)
        applyStaticFriction(collisionConstraints, dt, settings.dynamicFrictionCompliance)

        // Step 3, compute new velocities given the positional updates
        for (body in bodies)
            if (!body.isStatic) body.update(dt)

        // Step 4, solve velocity constraints
        velocityConstraints.forEach {
            it.reset()
        }
        solver.solveVelocityConstraints(velocityConstraints, settings.iterations, dt)

        // Step 4.5, add dynamic friction using a Gauss-Seidel, regardless of [solver].
        // (This gives us unconditional stability)
        applyDynamicFriction(collisionConstraints, dt, settings.dynamicFrictionCompliance)
    }
}

private fun applyDynamicFriction(collisions: List<CollisionConstraint>, dt: Double, compliance: Double = 0.0) {
    collisions.forEach { collision ->
        with(collision) {
            if (lambda == 0.0) {
                // If this contact wasn't used, then it didn't effect velocity so we don't need to correct anything
                return@with
            }
            // For each collision contact, set the relative velocity of the collision points on both bodies to 0
            val body0CollisionPosInGlobal = body0.pose.transform(Vector3d(body0ContactPosInBody0Coordinates))
            val body1CollisionPosInGlobal = body1.pose.transform(Vector3d(body1ContactPosInBody1Coordinates))

            // Compute the current velocity along normal
            val body0VelocityAtPoint = body0.getVelocityAt(body0CollisionPosInGlobal)
            val body1VelocityAtPoint = body1.getVelocityAt(body1CollisionPosInGlobal)
            val relativeVelocity = body0VelocityAtPoint.sub(body1VelocityAtPoint, Vector3d())
            val relativeVelocityAlongNormal = contactNormalInGlobalCoordinates.dot(relativeVelocity)

            run applyDynamicFriction2@{
                val tangentialVelocity =
                    Vector3d(relativeVelocity).fma(-relativeVelocityAlongNormal, contactNormalInGlobalCoordinates)

                // v_t
                val tangentialVelocityLength = tangentialVelocity.length()

                // Avoid dividing by 0
                if (tangentialVelocityLength < 1e-12) return@applyDynamicFriction2

                // u_d
                val dynamicFrictionCoefficient =
                    (body0.dynamicFrictionCoefficient + body1.dynamicFrictionCoefficient) / 2.0

                // f_d
                val normalForce = abs(lambda) / (dt * dt)

                val friction =
                    tangentialVelocity.mul(
                        -min(
                            dt * dynamicFrictionCoefficient * normalForce, tangentialVelocityLength
                        ) / tangentialVelocityLength
                    )

                applyBodyPairCorrection(
                    body0, body1, friction, compliance, dt, body0CollisionPosInGlobal,
                    body1CollisionPosInGlobal,
                    true
                )
            }
        }
    }
}

/**
 * We solve this after all the other position constraints have solved because these calculations require the collision
 * lambda.
 */
private fun applyStaticFriction(collisions: List<CollisionConstraint>, dt: Double, compliance: Double = 0.0) {
    collisions.forEach { collision ->
        with(collision) {
            if (lambda == 0.0) {
                // If this contact wasn't used, then it didn't effect velocity so we don't need to correct anything
                return@with
            }

            val body0PointPosInGlobal = body0.pose.transform(Vector3d(body0ContactPosInBody0Coordinates))
            val body1PointPosInGlobal = body1.pose.transform(Vector3d(body1ContactPosInBody1Coordinates))

            // Next, apply static friction
            val staticFrictionCoefficient =
                (body0.staticFrictionCoefficient + body1.staticFrictionCoefficient) / 2.0

            val prevBody0PointPosInGlobal = body0.prevPose.transform(Vector3d(body0ContactPosInBody0Coordinates))
            val prevBody1PointPosInGlobal = body1.prevPose.transform(Vector3d(body1ContactPosInBody1Coordinates))

            val relativeMotionOfContactPoints =
                Vector3d(body0PointPosInGlobal).sub(prevBody0PointPosInGlobal).sub(body1PointPosInGlobal)
                    .add(prevBody1PointPosInGlobal)

            val tangentialRelativeMotionOfContactPoints =
                Vector3d(relativeMotionOfContactPoints).fma(
                    -relativeMotionOfContactPoints.dot(contactNormalInGlobalCoordinates),
                    contactNormalInGlobalCoordinates
                )

            tangentialRelativeMotionOfContactPoints.mul(-1.0)

            val maxStaticFrictionLambda = abs(lambda * staticFrictionCoefficient)

            val tangentialLambdaThisSubStep = applyBodyPairCorrection(
                body0 = body0,
                body1 = body1,
                corr = tangentialRelativeMotionOfContactPoints,
                compliance = compliance,
                dt = dt,
                pos0 = body0PointPosInGlobal,
                pos1 = body1PointPosInGlobal,
                velocityLevel = false,
                prevLambda = 0.0,
                maxLambda = maxStaticFrictionLambda
            )
        }
    }
}

private data class CollisionData(val body0: Body, val body1: Body, val collisionResult: CollisionResult)

private fun generateCollisions(
    bodies: List<Body>, dt: Double, speculativeContactDistance: Double = 0.05
): List<CollisionData> {
    val collisionDataList = ArrayList<CollisionData>()
    for (i in bodies.indices) {
        for (j in i + 1 until bodies.size) {
            val body0: Body
            val body1: Body
            if (bodies[i].shape.sortIndex >= bodies[j].shape.sortIndex) {
                body0 = bodies[i]
                body1 = bodies[j]
            } else {
                body0 = bodies[j]
                body1 = bodies[i]
            }

            if (body0.isStatic and body1.isStatic) {
                continue // Both bodies are static, don't bother to collide with both of them
            }

            val collisionResult =
                ColliderResolver.computeCollisionBetweenShapes(
                    body0.shape, body0.pose, body0.vel, body0.omega, body1.shape, body1.pose, body1.vel, body1.omega,
                    dt, speculativeContactDistance
                )

            if (collisionResult != null && collisionResult.collisionPoints.isNotEmpty()) {
                collisionDataList.add(CollisionData(body0, body1, collisionResult))
            }
        }
    }
    return collisionDataList
}
