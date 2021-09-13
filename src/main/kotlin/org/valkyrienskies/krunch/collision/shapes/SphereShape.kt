package org.valkyrienskies.krunch.collision.shapes

import org.joml.primitives.AABBd

data class SphereShape(val radius: Double) : CollisionShape {
    override val sortIndex: Int = 2

    override fun getAABB(dest: AABBd): AABBd {
        dest.minX = -radius
        dest.minY = -radius
        dest.minZ = -radius
        dest.maxX = radius
        dest.maxY = radius
        dest.maxZ = radius
        return dest
    }
}
