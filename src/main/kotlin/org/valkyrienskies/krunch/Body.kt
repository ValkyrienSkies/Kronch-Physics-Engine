package org.valkyrienskies.krunch

import org.joml.Quaterniond
import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.krunch.collision.shapes.CollisionShape
import org.valkyrienskies.krunch.collision.shapes.SphereShape
import kotlin.math.abs

class Body(_pose: Pose) {

    val pose = _pose.clone()
    val prevPose = _pose.clone()

    val vel = Vector3d()
    val omega = Vector3d()

    // [prevVel] and [prevOmega] are used to compute friction
    internal val prevVel = Vector3d()
    internal val prevOmega = Vector3d()

    var invMass = 1.0
    val invInertia = Vector3d(1.0, 1.0, 1.0)

    var isStatic = false

    // Use a sphere shape by default
    var shape: CollisionShape = SphereShape(.5)
    var coefficientOfRestitution = .5
    var staticFrictionCoefficient = 1.0
    var dynamicFrictionCoefficient = .4

    fun setBox(size: Vector3d, density: Double = 1.0) {
        var mass = size.x * size.y * size.z * density
        invMass = 1.0 / mass
        mass /= 12.0

        invInertia.set(
            1.0 / (size.y * size.y + size.z * size.z) / mass,
            1.0 / (size.z * size.z + size.x * size.x) / mass,
            1.0 / (size.x * size.x + size.y * size.y) / mass
        )
    }

    fun applyRotation(rot: Vector3dc, scale: Double = 1.0) {
        var scaleConsideringMaxRotation = scale
        // safety clamping. This happens very rarely if the solver
        // wants to turn the body by more than 30 degrees in the
        // orders of milliseconds

        val maxPhi = 0.5
        val phi = rot.length()

        if (phi * scaleConsideringMaxRotation > maxRotationPerSubstep)
            scaleConsideringMaxRotation = maxRotationPerSubstep / phi

        val dq = Quaterniond(
            rot.x() * scaleConsideringMaxRotation, rot.y() * scaleConsideringMaxRotation,
            rot.z() * scaleConsideringMaxRotation, 0.0
        )
        dq.mul(this.pose.q)
        this.pose.q.set(
            this.pose.q.x + 0.5 * dq.x, this.pose.q.y + 0.5 * dq.y,
            this.pose.q.z + 0.5 * dq.z, this.pose.q.w + 0.5 * dq.w
        )
        this.pose.q.normalize()
    }

    fun integrate(dt: Double, gravity: Vector3dc) {
        this.prevPose.set(this.pose)
        this.vel.add(gravity.x() * dt, gravity.y() * dt, gravity.z() * dt)
        this.pose.p.add(this.vel.x * dt, this.vel.y * dt, this.vel.z * dt)
        this.applyRotation(this.omega, dt)
    }

    fun update(dt: Double) {
        this.prevVel.set(this.vel)
        this.prevOmega.set(this.omega)

        this.pose.p.sub(this.prevPose.p, this.vel)

        this.vel.mul(1.0 / dt)

        val dq = this.pose.q.mul(Quaterniond(this.prevPose.q).conjugate(), Quaterniond())

        dq.normalize()

        if (abs(dq.x) > 1e-10) {
            val j = 1
        }

        this.omega.set(dq.x * 2.0 / dt, dq.y * 2.0 / dt, dq.z * 2.0 / dt)
        if (dq.w < 0.0)
            this.omega.set(-this.omega.x, -this.omega.y, -this.omega.z)

        // this.omega.mul(1.0 - 1.0 * dt)
        // this.vel.mul(1.0 - 1.0 * dt)
    }

    fun getVelocityAt(pos: Vector3dc): Vector3d {
        // if (isStatic) return Vector3d()
        val vel = Vector3d(0.0, 0.0, 0.0)
        pos.sub(this.pose.p, vel)
        vel.cross(this.omega)
        this.vel.sub(vel, vel)
        return vel
    }

    fun getPrevVelocityAt(pos: Vector3dc): Vector3d {
        // if (isStatic) return Vector3d()
        val vel = Vector3d(0.0, 0.0, 0.0)
        pos.sub(this.pose.p, vel)
        vel.cross(this.prevOmega)
        this.prevVel.sub(vel, vel)
        return vel
    }

    fun getInverseMass(normal: Vector3dc, pos: Vector3dc? = null): Double {
        if (isStatic) throw IllegalStateException("Cannot get inverse mass of a static body")
        val n = Vector3d()
        if (pos == null)
            n.set(normal)
        else {
            pos.sub(this.pose.p, n)
            n.cross(normal)
        }
        this.pose.invRotate(n)
        var w = n.x * n.x * this.invInertia.x +
            n.y * n.y * this.invInertia.y +
            n.z * n.z * this.invInertia.z
        if (pos != null)
            w += this.invMass
        return w
    }

    fun applyCorrection(corr: Vector3dc, pos: Vector3dc? = null, velocityLevel: Boolean = false) {
        val dq = Vector3d()
        if (pos == null)
            dq.set(corr)
        else {
            if (velocityLevel)
                this.vel.add(corr.x() * this.invMass, corr.y() * this.invMass, corr.z() * this.invMass)
            else
                this.pose.p.add(corr.x() * this.invMass, corr.y() * this.invMass, corr.z() * this.invMass)
            pos.sub(this.pose.p, dq)
            dq.cross(corr)
        }
        this.pose.invRotate(dq)
        dq.set(
            this.invInertia.x * dq.x,
            this.invInertia.y * dq.y, this.invInertia.z * dq.z
        )
        this.pose.rotate(dq)
        if (velocityLevel)
            this.omega.add(dq)
        else
            this.applyRotation(dq)
    }

    /**
     * Returns the impulse that will adjust the position/velocity for constraints.
     */
    inline fun getCorrectionImpulses(
        corr: Vector3dc, pos: Vector3dc? = null,
        function: (linearImpulse: Vector3dc?, angularImpulse: Vector3dc?) -> Unit
    ) {
        var linearImpulse: Vector3dc? = null
        var angularImpulse: Vector3dc? = null

        val dq = Vector3d()
        if (pos == null)
            dq.set(corr)
        else {
            // this.pose.p.add(corr.x() * this.invMass, corr.y() * this.invMass, corr.z() * this.invMass)
            linearImpulse =
                Vector3d(corr.x() * this.invMass, corr.y() * this.invMass, corr.z() * this.invMass)
            pos.sub(this.pose.p, dq)
            dq.cross(corr)
        }
        this.pose.invRotate(dq)
        dq.set(
            this.invInertia.x * dq.x,
            this.invInertia.y * dq.y, this.invInertia.z * dq.z
        )
        this.pose.rotate(dq)

        angularImpulse = dq

        // this.applyRotation(dq)

        function(linearImpulse, angularImpulse)
    }

    companion object {
        fun createStaticBody(pose: Pose, shape: CollisionShape, coefficientOfRestitution: Double = .8): Body {
            val body = Body(pose)
            body.shape = shape
            body.isStatic = true
            body.coefficientOfRestitution = coefficientOfRestitution
            return body
        }
    }
}
