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
    fun rotate(v: Vector3d): Vector3d
    fun invRotate(v: Vector3d)
    fun transform(v: Vector3d): Vector3d
    fun invTransform(v: Vector3d): Vector3d
}
