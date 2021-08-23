package org.valkyrienskies.krunch.collision.shapes

import org.valkyrienskies.krunch.Pose

class CombinedShape(val collisionShapes: List<Pair<CollisionShape, Pose>>) : CollisionShape {
    override val sortIndex: Int = -1
}
