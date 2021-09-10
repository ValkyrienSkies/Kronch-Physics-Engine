package org.valkyrienskies.krunch

import org.joml.Quaterniond
import org.joml.Vector3d
import org.joml.Vector3dc

class Pose(
    override val p: Vector3d = Vector3d(0.0, 0.0, 0.0),
    override val q: Quaterniond = Quaterniond(0.0, 0.0, 0.0, 1.0)
) : Posec {

    fun set(pose: Pose) {
        this.p.set(pose.p)
        this.q.set(pose.q)
    }

    override fun clone() = Pose(Vector3d(p), Quaterniond(q))

    fun transformPose(pose: Pose) {
        pose.q.premul(this.q)
        this.rotate(pose.p)
        pose.p.add(this.p)
    }

    /**
     * Integrate this [Pose] with the given linear and angular velocities.
     */
    fun integrate(vel: Vector3dc, omega: Vector3dc, dt: Double): Pose {
        p.fma(dt, vel)
        val dq = Quaterniond(omega.x(), omega.y(), omega.z(), 0.0)
        dq.mul(q)
        q.set(
            q.x + dt * 0.5 * dq.x, q.y + dt * 0.5 * dq.y,
            q.z + dt * 0.5 * dq.z, q.w + dt * 0.5 * dq.w
        )
        q.normalize()
        return this
    }

    companion object {
        fun copyPose(pose: Posec): Pose = Pose(Vector3d(pose.p), Quaterniond(pose.q))
    }
}
