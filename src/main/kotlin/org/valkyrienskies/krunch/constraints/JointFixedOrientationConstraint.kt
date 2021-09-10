package org.valkyrienskies.krunch.constraints

import org.joml.Quaterniond
import org.joml.Quaterniondc
import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Joint

class JointFixedOrientationConstraint(private val joint: Joint) :
    RotationPositionConstraint(joint.body0, joint.body1, joint.swingLimitsCompliance) {

    override fun computeRotationCorrection(): Vector3dc {
        val body0Rotation: Quaterniondc = joint.body0?.pose?.q ?: Quaterniond()
        val body1Rotation: Quaterniondc = joint.body1?.pose?.q ?: Quaterniond()

        val q = Quaterniond(body0Rotation)
        q.conjugate()
        q.premul(body1Rotation)
        val omega = Vector3d()
        omega.set(2.0 * q.x, 2.0 * q.y, 2.0 * q.z)
        return omega
    }
}
