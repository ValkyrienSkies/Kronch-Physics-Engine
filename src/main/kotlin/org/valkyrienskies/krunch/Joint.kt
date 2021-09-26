package org.valkyrienskies.krunch

import org.joml.Vector3d
import org.valkyrienskies.krunch.JointType.FIXED
import org.valkyrienskies.krunch.JointType.HINGE
import org.valkyrienskies.krunch.JointType.SPHERICAL
import org.valkyrienskies.krunch.constraints.JointAttachmentConstraint
import org.valkyrienskies.krunch.constraints.JointFixedOrientationConstraint
import org.valkyrienskies.krunch.constraints.JointHingeOrientationConstraint
import org.valkyrienskies.krunch.constraints.JointHingeSwingLimitConstraint
import org.valkyrienskies.krunch.constraints.JointSphericalSwingLimitConstraint
import org.valkyrienskies.krunch.constraints.JointSphericalTwistLimitConstraint
import org.valkyrienskies.krunch.constraints.PositionConstraint
import kotlin.math.min

class Joint(
    val type: JointType,
    val body0: Body?,
    val body1: Body?,
    _localPose0: Pose,
    _localPose1: Pose,
    val compliance: Double = 1e-4,
    var rotDamping: Double = 0.0,
    var posDamping: Double = 0.0,
    val hasSwingLimits: Boolean = false,
    val minSwingAngle: Double = -2.0 * Math.PI,
    val maxSwingAngle: Double = 2.0 * Math.PI,
    val swingLimitsCompliance: Double = 0.0,
    val hasTwistLimits: Boolean = false,
    val minTwistAngle: Double = -2.0 * Math.PI,
    val maxTwistAngle: Double = 2.0 * Math.PI,
    val twistLimitCompliance: Double = 0.0
) {
    val localPose0: Pose = _localPose0.clone()
    val localPose1: Pose = _localPose1.clone()
    private val globalPose0: Pose = _localPose0.clone()
    private val globalPose1: Pose = _localPose1.clone()

    private val positionConstraints: List<PositionConstraint>

    init {
        val positionConstraintsMutable = ArrayList<PositionConstraint>()
        when (type) {
            FIXED -> positionConstraintsMutable.add(JointFixedOrientationConstraint(this))
            HINGE -> {
                positionConstraintsMutable.add(JointHingeOrientationConstraint(this))
                if (hasSwingLimits) positionConstraintsMutable.add(JointHingeSwingLimitConstraint(this))
            }
            SPHERICAL -> {
                if (hasSwingLimits) positionConstraintsMutable.add(JointSphericalSwingLimitConstraint(this))
                if (hasTwistLimits) positionConstraintsMutable.add(JointSphericalTwistLimitConstraint(this))
            }
        }
        positionConstraintsMutable.add(JointAttachmentConstraint(this))
        positionConstraints = positionConstraintsMutable
    }

    private fun updateGlobalPoses() {
        this.globalPose0.set(this.localPose0)
        if (this.body0 != null)
            this.body0.pose.transformPose(this.globalPose0)
        this.globalPose1.set(this.localPose1)
        if (this.body1 != null)
            this.body1.pose.transformPose(this.globalPose1)
    }

    fun solveVel(dt: Double) {
        // Gauss-Seidel vals us make damping unconditionally stable in a
        // very simple way. We clamp the correction for each constraint
        // to the magnitude of the currect velocity making sure that
        // we never subtract more than there actually is.

        if (this.rotDamping > 0.0) {
            val omega = Vector3d(0.0, 0.0, 0.0)
            if (this.body0 != null)
                omega.sub(this.body0.omega)
            if (this.body1 != null)
                omega.add(this.body1.omega)
            omega.mul(min(1.00, this.rotDamping * dt))
            applyBodyPairCorrection(
                this.body0, this.body1, omega, 0.0, dt,
                null, null, true
            )
        }
        if (this.posDamping > 0.0) {
            this.updateGlobalPoses()
            val vel = Vector3d(0.0, 0.0, 0.0)
            if (this.body0 != null)
                vel.sub(this.body0.getVelocityAt(this.globalPose0.p))
            if (this.body1 != null)
                vel.add(this.body1.getVelocityAt(this.globalPose1.p))
            vel.mul(min(1.0, this.posDamping * dt))
            applyBodyPairCorrection(
                this.body0, this.body1, vel, 0.0, dt,
                this.globalPose0.p, this.globalPose1.p, true
            )
        }
    }

    fun getPositionConstraints(): List<PositionConstraint> = positionConstraints

    companion object {
        fun createJoint(
            type: JointType,
            body0: Body?,
            body1: Body?,
            _localPose0: Pose,
            _localPose1: Pose
        ): Joint = Joint(type, body0, body1, _localPose0, _localPose1)
    }
}
