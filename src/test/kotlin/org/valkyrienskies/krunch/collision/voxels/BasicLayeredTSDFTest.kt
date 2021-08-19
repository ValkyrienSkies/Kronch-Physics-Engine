package org.valkyrienskies.krunch.collision.voxels

import org.joml.Vector3d
import org.joml.Vector3dc
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

internal class BasicLayeredTSDFTest {

    @Test
    fun getClosestPoint() {
        val singleVoxelLayeredTSDF = BasicLayeredTSDF()
        singleVoxelLayeredTSDF.setVoxel(0, 0, 0, true)

        val testPos: Vector3dc = Vector3d(0.0, 1.0, 0.0)

        val closestPoint = Vector3d()
        val validQuery = singleVoxelLayeredTSDF.getClosestPoint(testPos.x(), testPos.y(), testPos.z(), closestPoint)

        assertTrue(validQuery)
        assertEquals(Vector3d(0.0, 0.5, 0.0), closestPoint)
    }
}
