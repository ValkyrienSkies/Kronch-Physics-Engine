package org.valkyrienskies.krunch

import org.joml.Vector3d
import org.valkyrienskies.krunch.JointType.FIXED
import org.valkyrienskies.krunch.JointType.HINGE
import org.valkyrienskies.krunch.JointType.SPHERICAL
import kotlin.math.min

class Joint(
    val type: JointType,
    val body0: Body?,
    val body1: Body?,
    _localPose0: Pose,
    _localPose1: Pose
) {

    val localPose0: Pose
    val localPose1: Pose
    val globalPose0: Pose
    val globalPose1: Pose

    val compliance = 0.0
    var rotDamping = 0.0
    var posDamping = 0.0
    val hasSwingLimits = false
    val minSwingAngle = -2.0 * Math.PI
    val maxSwingAngle = 2.0 * Math.PI
    val swingLimitsCompliance = 0.0
    val hasTwistLimits = false
    val minTwistAngle = -2.0 * Math.PI
    val maxTwistAngle = 2.0 * Math.PI
    val twistLimitCompliance = 0.0

    init {
        this.localPose0 = _localPose0.clone()
        this.localPose1 = _localPose1.clone()
        this.globalPose0 = _localPose0.clone()
        this.globalPose1 = _localPose1.clone()
    }

    fun updateGlobalPoses() {
        this.globalPose0.copy(this.localPose0)
        if (this.body0 != null)
            this.body0.pose.transformPose(this.globalPose0)
        this.globalPose1.copy(this.localPose1)
        if (this.body1 != null)
            this.body1.pose.transformPose(this.globalPose1)
    }

    fun solvePos(dt: Double) {

        this.updateGlobalPoses()

        // orientation

        if (this.type == FIXED) {
            val q = globalPose0.q
            q.conjugate()
            q.premul(globalPose1.q)
            val omega = Vector3d()
            omega.set(2.0 * q.x, 2.0 * q.y, 2.0 * q.z)
            // todo: wtf
            // if (omega.w < 0.0)
            //     omega.mul(-1.0)
            applyBodyPairCorrection(body0, body1, omega, this.compliance, dt)
        }

        if (this.type == HINGE) {

            // align axes
            val a0 = getQuatAxis0(this.globalPose0.q)
            val b0 = getQuatAxis1(this.globalPose0.q)
            val c0 = getQuatAxis2(this.globalPose0.q)
            val a1 = getQuatAxis0(this.globalPose1.q)
            a0.cross(a1)
            applyBodyPairCorrection(this.body0, this.body1, a0, 0.0, dt)

            // limits
            if (this.hasSwingLimits) {
                this.updateGlobalPoses()
                val n = getQuatAxis0(this.globalPose0.q)
                val b0 = getQuatAxis1(this.globalPose0.q)
                val b1 = getQuatAxis1(this.globalPose1.q)
                limitAngle(
                    this.body0, this.body1, n, b0, b1,
                    this.minSwingAngle, this.maxSwingAngle, this.swingLimitsCompliance, dt
                )
            }
        }

        if (this.type == SPHERICAL) {

            // swing limits
            if (this.hasSwingLimits) {
                this.updateGlobalPoses()
                val a0 = getQuatAxis0(this.globalPose0.q)
                val a1 = getQuatAxis0(this.globalPose1.q)
                val n = a0.cross(a1, Vector3d())
                n.normalize()
                limitAngle(
                    this.body0, this.body1, n, a0, a1,
                    this.minSwingAngle, this.maxSwingAngle, this.swingLimitsCompliance, dt
                )
            }
            // twist limits
            if (this.hasTwistLimits) {
                this.updateGlobalPoses()
                val n0 = getQuatAxis0(this.globalPose0.q)
                val n1 = getQuatAxis0(this.globalPose1.q)
                val n = n0.add(n1, Vector3d())
                n.normalize()

                val a0 = getQuatAxis1(this.globalPose0.q)
                val scale0 = -n.dot(a0)
                a0.add(n.x * scale0, n.y * scale0, n.z * scale0)
                a0.normalize()

                val a1 = getQuatAxis1(this.globalPose1.q)
                val scale1 = -n.dot(a1)

                a1.add(n.x * scale1, n.y * scale1, n.z * scale1)
                a1.normalize()

                // handling gimbal lock problem
                val maxCorr = if (n0.dot(n1) > -0.5) 2.0 * Math.PI else 1.0 * dt

                limitAngle(
                    this.body0, this.body1, n, a0, a1,
                    this.minTwistAngle, this.maxTwistAngle, this.twistLimitCompliance, dt, maxCorr
                )
            }
        }

        // position

        // simple attachment

        this.updateGlobalPoses()
        val corr = this.globalPose1.p.sub(this.globalPose0.p, Vector3d())
        applyBodyPairCorrection(
            this.body0, this.body1, corr, this.compliance, dt,
            this.globalPose0.p, this.globalPose1.p
        )
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
}
