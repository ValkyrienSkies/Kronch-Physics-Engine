package org.valkyrienskies.krunch.constraints

import org.joml.Quaterniond
import org.joml.Quaterniondc
import org.joml.Vector3dc
import org.valkyrienskies.krunch.Joint
import org.valkyrienskies.krunch.getQuaternionAxis0

class JointHingeOrientationConstraint(private val joint: Joint) :
    RotationPositionConstraint(joint.body0, joint.body1, joint.swingLimitsCompliance) {

    override fun computeRotationCorrection(): Vector3dc {
        val body0Rotation: Quaterniondc = joint.body0?.pose?.q ?: Quaterniond()
        val body1Rotation: Quaterniondc = joint.body1?.pose?.q ?: Quaterniond()
        val a0 = getQuaternionAxis0(body0Rotation)
        val a1 = getQuaternionAxis0(body1Rotation)
        a0.cross(a1)
        return a0
    }
}
