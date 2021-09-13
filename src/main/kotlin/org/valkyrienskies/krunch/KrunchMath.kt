package org.valkyrienskies.krunch

import org.joml.Math
import org.joml.Quaterniondc
import org.joml.Vector3d
import org.joml.Vector3dc
import org.joml.primitives.AABBd
import org.joml.primitives.AABBdc
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

fun getQuaternionAxis0(q: Quaterniondc): Vector3d {
    val x2 = q.x() * 2.0
    val w2 = q.w() * 2.0
    return Vector3d((q.w() * w2) - 1.0 + q.x() * x2, (q.z() * w2) + q.y() * x2, (-q.y() * w2) + q.z() * x2)
}

fun getQuaternionAxis1(q: Quaterniondc): Vector3d {
    val y2 = q.y() * 2.0
    val w2 = q.w() * 2.0
    return Vector3d((-q.z() * w2) + q.x() * y2, (q.w() * w2) - 1.0 + q.y() * y2, (q.x() * w2) + q.z() * y2)
}

fun getQuaternionAxis2(q: Quaterniondc): Vector3d {
    val z2 = q.z() * 2.0
    val w2 = q.w() * 2.0
    return Vector3d((q.y() * w2) + q.x() * z2, (-q.x() * w2) + q.y() * z2, (q.w() * w2) - 1.0 + q.z() * z2)
}

fun AABBd.transform(pose: Posec): AABBd = transform(pose, this)

fun AABBdc.transform(pose: Posec, dest: AABBd): AABBd {
    val dx = maxX() - minX()
    val dy = maxY() - minY()
    val dz = maxZ() - minZ()
    var minx = Double.POSITIVE_INFINITY
    var miny = Double.POSITIVE_INFINITY
    var minz = Double.POSITIVE_INFINITY
    var maxx = Double.NEGATIVE_INFINITY
    var maxy = Double.NEGATIVE_INFINITY
    var maxz = Double.NEGATIVE_INFINITY
    for (i in 0..7) {
        val x = minX() + (i and 1) * dx
        val y = minY() + (i shr 1 and 1) * dy
        val z = minZ() + (i shr 2 and 1) * dz

        val xx: Double = pose.q.x() * pose.q.x()
        val yy: Double = pose.q.y() * pose.q.y()
        val zz: Double = pose.q.z() * pose.q.z()
        val ww: Double = pose.q.w() * pose.q.w()
        val xy: Double = pose.q.x() * pose.q.y()
        val xz: Double = pose.q.x() * pose.q.z()
        val yz: Double = pose.q.y() * pose.q.z()
        val xw: Double = pose.q.x() * pose.q.w()
        val zw: Double = pose.q.z() * pose.q.w()
        val yw: Double = pose.q.y() * pose.q.w()
        val k = 1 / (xx + yy + zz + ww)
        val tx: Double =
            Math.fma((xx - yy - zz + ww) * k, x, Math.fma(2 * (xy - zw) * k, y, 2 * (xz + yw) * k * z) + pose.p.x())
        val ty: Double =
            Math.fma(2 * (xy + zw) * k, x, Math.fma((yy - xx - zz + ww) * k, y, 2 * (yz - xw) * k * z) + pose.p.y())
        val tz: Double =
            Math.fma(2 * (xz - yw) * k, x, Math.fma(2 * (yz + xw) * k, y, (zz - xx - yy + ww) * k * z) + pose.p.z())

        minx = Math.min(tx, minx)
        miny = Math.min(ty, miny)
        minz = Math.min(tz, minz)
        maxx = Math.max(tx, maxx)
        maxy = Math.max(ty, maxy)
        maxz = Math.max(tz, maxz)
    }
    dest.minX = minx
    dest.minY = miny
    dest.minZ = minz
    dest.maxX = maxx
    dest.maxY = maxy
    dest.maxZ = maxz
    return dest
}

fun AABBd.extend(extend: Vector3dc): AABBd = extend(extend, this)

fun AABBdc.extend(extend: Vector3dc, dest: AABBd): AABBd {
    if (extend.x() > 0) dest.maxX += extend.x() else dest.minX += extend.x()
    if (extend.y() > 0) dest.maxY += extend.y() else dest.minY += extend.y()
    if (extend.z() > 0) dest.maxZ += extend.z() else dest.minZ += extend.z()
    return dest
}

fun AABBd.expand(expansion: Double): AABBd = expand(expansion, expansion, expansion, this)

fun AABBd.expand(x: Double, y: Double, z: Double): AABBd = expand(x, y, z, this)

fun AABBdc.expand(x: Double, y: Double, z: Double, dest: AABBd): AABBd {
    dest.minX -= x
    dest.minY -= y
    dest.minZ -= z
    dest.maxX += x
    dest.maxY += y
    dest.maxZ += z
    return dest
}
