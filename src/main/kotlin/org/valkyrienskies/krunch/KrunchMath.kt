package org.valkyrienskies.krunch

import org.joml.Vector3d
import org.joml.Vector3dc
import kotlin.math.min

fun getVelocityAt(
    localPos: Vector3dc,
    velocity: Vector3dc,
    angularVelocity: Vector3dc,
    dest: Vector3d = Vector3d()
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
    body0Pose: Posec,
    body0Velocity: Vector3dc,
    body0AngularVelocity: Vector3dc,
    body1Pose: Posec,
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
