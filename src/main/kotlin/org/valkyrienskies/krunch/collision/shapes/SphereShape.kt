package org.valkyrienskies.krunch.collision.shapes

data class SphereShape(val radius: Double) : CollisionShape {
    override val sortIndex: Int = 2
}
