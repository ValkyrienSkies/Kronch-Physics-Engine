package org.valkyrienskies.krunch.collision.shapes

class BoxShape(val xRadius: Double, val yRadius: Double, val zRadius: Double) : CollisionShape {
    override val sortIndex: Int = 1
}
