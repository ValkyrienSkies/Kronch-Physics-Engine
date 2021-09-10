package org.valkyrienskies.krunch.constraints

import org.joml.Quaterniond
import org.joml.Quaterniondc
import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Joint
import org.valkyrienskies.krunch.getQuaternionAxis0
import org.valkyrienskies.krunch.getQuaternionAxis1
import kotlin.math.asin
import kotlin.math.max
import kotlin.math.min

class JointSphericalTwistLimitConstraint(private val joint: Joint) :
    RotationPositionConstraint(joint.body0, joint.body1, joint.twistLimitCompliance) {

    override fun computeRotationCorrection(): Vector3dc {
        // TODO: Set this to the real dt
        val dt = 1.0 / (60.0 * 20.0)

        val body0Rotation: Quaterniondc = joint.body0?.pose?.q ?: Quaterniond()
        val body1Rotation: Quaterniondc = joint.body1?.pose?.q ?: Quaterniond()

        val n0 = getQuaternionAxis0(body0Rotation)
        val n1 = getQuaternionAxis0(body1Rotation)
        val n = n0.add(n1, Vector3d())

        if (n.lengthSquared() < 1e-24) return Vector3d()

        n.normalize()

        val a = getQuaternionAxis1(body0Rotation)
        val scale0 = -n.dot(a)
        a.add(n.x * scale0, n.y * scale0, n.z * scale0)

        if (a.lengthSquared() < 1e-24) return Vector3d()

        a.normalize()

        val b = getQuaternionAxis1(body1Rotation)
        val scale1 = -n.dot(b)

        b.add(n.x * scale1, n.y * scale1, n.z * scale1)

        if (b.lengthSquared() < 1e-24) return Vector3d()
        b.normalize()

        // handling gimbal lock problem
        val maxCorr = if (n0.dot(n1) > -0.5) 2.0 * Math.PI else 1.0 * dt

        // the key function to handle all angular joint limits
        val c = a.cross(b, Vector3d())

        var phi = asin(c.dot(n))
        if (a.dot(b) < 0.0)
            phi = Math.PI - phi

        if (phi > Math.PI)
            phi -= 2.0 * Math.PI
        if (phi < -Math.PI)
            phi += 2.0 * Math.PI

        if (phi < joint.minTwistAngle || phi > joint.maxTwistAngle) {
            phi = min(phi, joint.maxTwistAngle)
            phi = max(phi, joint.minTwistAngle)

            val q = Quaterniond()
            q.setAngleAxis(phi, n)

            val omega = Vector3d(a)
            omega.rotate(q)
            omega.cross(b)

            phi = omega.length()
            if (phi > maxCorr)
                omega.mul(maxCorr / phi)

            return omega
        }

        return Vector3d()
    }
}
