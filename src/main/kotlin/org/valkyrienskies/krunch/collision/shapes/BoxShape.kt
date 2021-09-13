package org.valkyrienskies.krunch.collision.shapes

import org.joml.primitives.AABBd

class BoxShape(val xRadius: Double, val yRadius: Double, val zRadius: Double) : CollisionShape {
    override val sortIndex: Int = 1

    override fun getAABB(dest: AABBd): AABBd {
        dest.minX = -xRadius
        dest.minY = -yRadius
        dest.minZ = -zRadius
        dest.maxX = xRadius
        dest.maxY = yRadius
        dest.maxZ = zRadius
        return dest
    }
}
