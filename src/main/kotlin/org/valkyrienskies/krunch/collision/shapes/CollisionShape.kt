package org.valkyrienskies.krunch.collision.shapes

import org.joml.primitives.AABBd

/**
 * The shape of an object in the physics engine.
 */
interface CollisionShape {
    val sortIndex: Int
    fun getAABB(dest: AABBd = AABBd()): AABBd
}
