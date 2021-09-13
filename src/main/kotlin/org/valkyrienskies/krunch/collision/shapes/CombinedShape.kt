package org.valkyrienskies.krunch.collision.shapes

import org.joml.primitives.AABBd
import org.valkyrienskies.krunch.Pose

class CombinedShape(val collisionShapes: List<Pair<CollisionShape, Pose>>) : CollisionShape {
    override val sortIndex: Int = -1
    override fun getAABB(dest: AABBd): AABBd {
        TODO("Not yet implemented")
    }
}
