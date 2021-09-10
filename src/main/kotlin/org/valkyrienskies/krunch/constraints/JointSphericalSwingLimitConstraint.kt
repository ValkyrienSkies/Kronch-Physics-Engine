package org.valkyrienskies.krunch.constraints

import org.joml.Quaterniond
import org.joml.Quaterniondc
import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Joint
import org.valkyrienskies.krunch.getQuaternionAxis0
import kotlin.math.asin
import kotlin.math.max
import kotlin.math.min

class JointSphericalSwingLimitConstraint(private val joint: Joint) :
    RotationPositionConstraint(joint.body0, joint.body1, joint.swingLimitsCompliance) {

    override fun computeRotationCorrection(): Vector3dc {
        val body0Rotation: Quaterniondc = joint.body0?.pose?.q ?: Quaterniond()
        val body1Rotation: Quaterniondc = joint.body1?.pose?.q ?: Quaterniond()

        val a = getQuaternionAxis0(body0Rotation)
        val b = getQuaternionAxis0(body1Rotation)
        val n = a.cross(b, Vector3d())

        if (n.lengthSquared() < 1e-24) return Vector3d()

        n.normalize()

        // the key function to handle all angular joint limits
        val c = a.cross(b, Vector3d())

        var phi = asin(c.dot(n))
        if (a.dot(b) < 0.0)
            phi = Math.PI - phi

        if (phi > Math.PI)
            phi -= 2.0 * Math.PI
        if (phi < -Math.PI)
            phi += 2.0 * Math.PI

        val maxCorr: Double = Math.PI

        if (phi < joint.minSwingAngle || phi > joint.maxSwingAngle) {
            phi = min(phi, joint.maxSwingAngle)
            phi = max(phi, joint.minSwingAngle)

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
