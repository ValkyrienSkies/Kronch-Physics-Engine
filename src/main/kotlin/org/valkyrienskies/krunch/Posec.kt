package org.valkyrienskies.krunch

import org.joml.Quaterniondc
import org.joml.Vector3d
import org.joml.Vector3dc

/**
 * An immutable view of [Pose]
 */
interface Posec {
    val p: Vector3dc
    val q: Quaterniondc
    fun clone(): Pose

    fun rotate(v: Vector3d): Vector3d {
        v.rotate(this.q)
        return v
    }

    fun invRotate(v: Vector3d) {
        this.q.transformInverse(v)
    }

    fun transform(v: Vector3d): Vector3d {
        v.rotate(this.q)
        v.add(this.p)
        return v
    }

    fun invTransform(v: Vector3d): Vector3d {
        v.sub(this.p)
        this.invRotate(v)
        return v
    }
}
