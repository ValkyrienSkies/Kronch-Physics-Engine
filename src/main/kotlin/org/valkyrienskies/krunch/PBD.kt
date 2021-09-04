package org.valkyrienskies.krunch

import org.joml.Quaterniond
import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.collision.CollisionResult
import org.valkyrienskies.krunch.collision.colliders.ColliderResolver
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.min

// pretty much one-for-one port of https://github.com/matthias-research/pages/blob/master/challenges/PBD.js

fun getQuatAxis0(q: Quaterniond): Vector3d {
    val x2 = q.x * 2.0
    val w2 = q.w * 2.0
    return Vector3d((q.w * w2) - 1.0 + q.x * x2, (q.z * w2) + q.y * x2, (-q.y * w2) + q.z * x2)
}

fun getQuatAxis1(q: Quaterniond): Vector3d {
    val y2 = q.y * 2.0
    val w2 = q.w * 2.0
    return Vector3d((-q.z * w2) + q.x * y2, (q.w * w2) - 1.0 + q.y * y2, (q.x * w2) + q.z * y2)
}

fun getQuatAxis2(q: Quaterniond): Vector3d {
    val z2 = q.z * 2.0
    val w2 = q.w * 2.0
    return Vector3d((q.y * w2) + q.x * z2, (-q.x * w2) + q.y * z2, (q.w * w2) - 1.0 + q.z * z2)
}

const val maxRotationPerSubstep = 0.5

private const val PAIR_CORRECTION_MIN_LENGTH = 1e-10

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

    normal.mul(-deltaLambda)

    if (body0 != null && !body0.isStatic) {
        body0.applyCorrection(normal, pos0, velocityLevel)
    }

    if (body1 != null && !body1.isStatic) {
        normal.mul(-1.0)
        body1.applyCorrection(normal, pos1, velocityLevel)
    }
    return prevLambda + deltaLambda
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

fun simulate(bodies: List<Body>, joints: List<Joint>, timeStep: Double, numSubsteps: Int, gravity: Vector3dc) {
    val dt = timeStep / numSubsteps

    // Only solve collision detection once per time step
    val collisions = generateCollisionConstraints(bodies, timeStep)

    for (step in 0 until numSubsteps) {
        // Step 1, integrate velocity into position
        for (body in bodies)
            if (!body.isStatic) body.integrate(dt, gravity)

        // Step 2, solve positional constraints (like joints and contacts)
        for (joint in joints)
            joint.solvePos(dt)

        // Collide shapes with each other
        applySubStepToCollisions(collisions)
        resolveCollisions(collisions, dt)

        // Step 3, compute new velocities given the positional updates
        for (body in bodies)
            if (!body.isStatic) body.update(dt)

        // Step 3.5, update velocities to apply friction and collision restitution

        /**
         * Run the restitution correction step multiple times.
         *
         * Note that in the XPBD paper they say to only run it once, but I think this step has to be run at least a few
         * times to handle multiple bodies.
         *
         * It seems like 3 is about right, too many steps seems to take away too much momentum.
         *
         * Maybe I should use the Jacobian solver to solve these equations?
         */
        for (i in 1..3) correctRestitution(collisions, dt)

        // Only run this once
        applyDynamicFriction(collisions, dt)

        // Step 4, solve velocity constraints
        for (joint in joints)
            joint.solveVel(dt)
    }
}

private fun applySubStepToCollisions(collisions: List<CollisionData>) {
    collisions.forEach { collisionData ->
        collisionData.collisionResult.collisionPoints.forEach { collisionContact ->
            collisionContact.skipThisSubStep = false
            collisionContact.normalThisSubStep = Vector3d(collisionContact.originalCollisionNormal)
            collisionContact.usedThisSubStep = false
            collisionContact.normalLambdaThisSubStep = 0.0
            collisionContact.tangentialLambdaThisSubStep = 0.0
        }
    }
}

private fun correctRestitution(collisions: List<CollisionData>, dt: Double, restitutionCompliance: Double = 0.0) {
    collisions.forEach { collision ->
        with(collision) {
            collisionResult.collisionPoints.forEach { collisionContact ->
                with(collisionContact) collisionContact@{
                    if (usedThisSubStep) {
                        // For each collision contact, set the relative velocity of the collision points on both bodies to 0
                        val body0CollisionPosInGlobal = body0.pose.transform(Vector3d(positionInFirstBody))
                        val body1CollisionPosInGlobal = body1.pose.transform(Vector3d(positionInSecondBody))

                        // Compute the current velocity along normal
                        val body0VelocityAtPoint = body0.getVelocityAt(body0CollisionPosInGlobal)
                        val body1VelocityAtPoint = body1.getVelocityAt(body1CollisionPosInGlobal)

                        val relativeVelocity = body0VelocityAtPoint.sub(body1VelocityAtPoint, Vector3d())
                        val relativeVelocityAlongNormal = normalThisSubStep.dot(relativeVelocity) // v_n

                        // Compute the previous velocity along normal
                        val relativeVelocityAlongNormalPrev =
                            if (abs(relativeVelocityAlongNormal) > 2 * 10.0 * dt) {
                                val body0VelocityAtPointPrev = body0.getPrevVelocityAt(body0CollisionPosInGlobal)
                                val body1VelocityAtPointPrev = body1.getPrevVelocityAt(body1CollisionPosInGlobal)
                                val relativeVelocityPrev =
                                    body0VelocityAtPointPrev.sub(body1VelocityAtPointPrev, Vector3d())
                                normalThisSubStep.dot(relativeVelocityPrev)
                            } else {
                                0.0
                            }

                        // Take the average of the coefficients of restitution of both bodies
                        val coefficientOfRestitution =
                            (body0.coefficientOfRestitution + body1.coefficientOfRestitution) / 2.0

                        // [deltaVelocity] serves 2 purposes:
                        // The first is to remove velocity added by [collision]
                        // The second is to apply the coefficient of restitution to [collision]
                        val deltaVelocity = normalThisSubStep.mul(
                            -relativeVelocityAlongNormal +
                                min(-coefficientOfRestitution * relativeVelocityAlongNormalPrev, 0.0),
                            Vector3d()
                        )

                        applyBodyPairCorrection(
                            body0, body1, deltaVelocity, restitutionCompliance, dt, body0CollisionPosInGlobal,
                            body1CollisionPosInGlobal,
                            true
                        )
                    }
                }
            }
        }
    }
}

private fun applyDynamicFriction(collisions: List<CollisionData>, dt: Double, restitutionCompliance: Double = 0.0) {
    collisions.forEach { collision ->
        with(collision) {
            collisionResult.collisionPoints.forEach { collisionContact ->
                with(collisionContact) collisionContact@{
                    if (!usedThisSubStep) {
                        // If this contact wasn't used, then it didn't effect velocity so we don't need to correct anything
                        return@collisionContact
                    }
                    // For each collision contact, set the relative velocity of the collision points on both bodies to 0
                    val body0CollisionPosInGlobal = body0.pose.transform(Vector3d(positionInFirstBody))
                    val body1CollisionPosInGlobal = body1.pose.transform(Vector3d(positionInSecondBody))

                    // Compute the current velocity along normal
                    val body0VelocityAtPoint = body0.getVelocityAt(body0CollisionPosInGlobal)
                    val body1VelocityAtPoint = body1.getVelocityAt(body1CollisionPosInGlobal)
                    val relativeVelocity = body0VelocityAtPoint.sub(body1VelocityAtPoint, Vector3d())
                    val relativeVelocityAlongNormal = normalThisSubStep.dot(relativeVelocity)

                    run applyDynamicFriction2@{
                        val tangentialVelocity =
                            Vector3d(relativeVelocity).fma(-relativeVelocityAlongNormal, normalThisSubStep)

                        // v_t
                        val tangentialVelocityLength = tangentialVelocity.length()

                        // Avoid dividing by 0
                        if (tangentialVelocityLength < 1e-12) return@applyDynamicFriction2

                        // u_d
                        val dynamicFrictionCoefficient =
                            (body0.dynamicFrictionCoefficient + body1.dynamicFrictionCoefficient) / 2.0

                        // f_d
                        val normalForce = abs(collisionContact.normalLambdaThisSubStep) / (dt * dt)

                        val friction =
                            tangentialVelocity.mul(
                                -min(
                                    dt * dynamicFrictionCoefficient * normalForce, tangentialVelocityLength
                                ) / tangentialVelocityLength
                            )

                        applyBodyPairCorrection(
                            body0, body1, friction, restitutionCompliance, dt, body0CollisionPosInGlobal,
                            body1CollisionPosInGlobal,
                            true
                        )
                    }
                }
            }
        }
    }
}

private fun resolveCollisions(collisions: List<CollisionData>, dt: Double, collisionCompliance: Double = 0.0) {
    collisions.forEach { collision ->
        with(collision) {
            collisionResult.collisionPoints.forEach { collisionContact ->
                with(collisionContact) collisionContact@{
                    // Potentially skip this collision
                    if (collisionContact.skipThisSubStep) return@collisionContact

                    if (collisionContact.normalThisSubStep.lengthSquared() < .99) throw IllegalStateException(
                        "The collision normal " + collisionContact.normalThisSubStep + " is not normal!"
                    )

                    val body0PointPosInGlobal = body0.pose.transform(Vector3d(positionInFirstBody))
                    val body1PointPosInGlobal = body1.pose.transform(Vector3d(positionInSecondBody))

                    val positionDifference = body0PointPosInGlobal.sub(body1PointPosInGlobal, Vector3d())
                    val d = normalThisSubStep.dot(positionDifference)

                    if (d < PAIR_CORRECTION_MIN_LENGTH) {
                        // No longer colliding, skip this contact
                        // Technically any d value > 0 is valid, but don't allow values that are too small (< PAIR_CORRECTION_MIN_LENGTH) to avoid floating point errors
                        return@collisionContact
                    }

                    // This part doesn't make sense to me now, but it fixes a lot of problems to make [corr] negative
                    val corr = normalThisSubStep.mul(-d, Vector3d())

                    // Compliance is set to [collisionCompliance] to dampen collision forces.
                    normalLambdaThisSubStep = applyBodyPairCorrection(
                        body0, body1, corr, collisionCompliance, dt,
                        body0PointPosInGlobal, body1PointPosInGlobal, false, normalLambdaThisSubStep
                    )

                    // Next, apply static friction
                    val staticFrictionCoefficient =
                        (body0.staticFrictionCoefficient + body1.staticFrictionCoefficient) / 2.0

                    val prevBody0PointPosInGlobal = body0.prevPose.transform(Vector3d(positionInFirstBody))
                    val prevBody1PointPosInGlobal = body1.prevPose.transform(Vector3d(positionInSecondBody))

                    val relativeMotionOfContactPoints =
                        Vector3d(body0PointPosInGlobal).sub(prevBody0PointPosInGlobal).sub(body1PointPosInGlobal)
                            .add(prevBody1PointPosInGlobal)

                    val tangentialRelativeMotionOfContactPoints =
                        Vector3d(relativeMotionOfContactPoints).fma(
                            -relativeMotionOfContactPoints.dot(normalThisSubStep), normalThisSubStep
                        )

                    tangentialRelativeMotionOfContactPoints.mul(-1.0)

                    val maxStaticFrictionLambda = abs(normalLambdaThisSubStep * staticFrictionCoefficient)

                    tangentialLambdaThisSubStep = applyBodyPairCorrection(
                        body0 = body0,
                        body1 = body1,
                        corr = tangentialRelativeMotionOfContactPoints,
                        compliance = collisionCompliance,
                        dt = dt,
                        pos0 = body0PointPosInGlobal,
                        pos1 = body1PointPosInGlobal,
                        velocityLevel = false,
                        prevLambda = 0.0,
                        maxLambda = maxStaticFrictionLambda
                    )

                    usedThisSubStep = true
                }
            }
        }
    }
}

private data class CollisionData(val body0: Body, val body1: Body, val collisionResult: CollisionResult)

private fun generateCollisionConstraints(bodies: List<Body>, dt: Double): List<CollisionData> {
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
                    dt, 0.05
                )

            if (collisionResult != null && collisionResult.collisionPoints.isNotEmpty()) {
                collisionDataList.add(CollisionData(body0, body1, collisionResult))
            }
        }
    }
    return collisionDataList
}

fun getVelocityAt(
    localPos: Vector3dc, velocity: Vector3dc, angularVelocity: Vector3dc, dest: Vector3d = Vector3d()
): Vector3dc {
    dest.set(angularVelocity)
    dest.cross(localPos)
    dest.add(velocity)
    return dest
}

/**
 * Computes the relative velocity between the collision points along the normal.
 *
 * When contact points are getting closer this function returns a negative value, which can be added to the current
 * distance between contact points to generate speculative contacts.
 *
 * When contact points are getting further this function returns 0 to avoid us skipping contacts.
 */
fun computeRelativeVelocityBetweenCollisionPointsAlongNormal(
    normal: Vector3dc,
    body0CollisionPointInBody0Coordinates: Vector3dc,
    body1CollisionPointInBody1Coordinates: Vector3dc,
    body0Pose: Pose,
    body0Velocity: Vector3dc,
    body0AngularVelocity: Vector3dc,
    body1Pose: Pose,
    body1Velocity: Vector3dc,
    body1AngularVelocity: Vector3dc
): Double {
    val body0CollisionPointRotated = body0Pose.rotate(Vector3d(body0CollisionPointInBody0Coordinates))
    val body1CollisionPointRotated = body1Pose.rotate(Vector3d(body1CollisionPointInBody1Coordinates))

    val body0VelocityAlongNormal =
        getVelocityAt(body0CollisionPointRotated, body0Velocity, body0AngularVelocity)
    val body1VelocityAlongNormal =
        getVelocityAt(body1CollisionPointRotated, body1Velocity, body1AngularVelocity)

    // Add this to difference to get the future distance between contacts
    // FutureDistance = CurrentDistance + RelativeVelocity
    // If relativeVelocity < 0 then they get closer
    // If relativeVelocity > 0 then they get further
    return min((normal.dot(body1VelocityAlongNormal) - normal.dot(body0VelocityAlongNormal)), 0.0)
}
