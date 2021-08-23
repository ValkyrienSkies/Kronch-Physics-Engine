package org.valkyrienskies.krunch.collision.voxels

import org.joml.Vector3i
import org.joml.Vector3ic
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test
import kotlin.random.Random
import kotlin.random.nextInt

internal class LongDataWorldTest {

    @Test
    fun testSettingAndGetting() {
        val SET_DATA: Long = 420
        val UNSET_DATA: Long = 0

        val minX = 0
        val minY = -200
        val minZ = 0
        val maxX = 15
        val maxY = 255
        val maxZ = 15

        val longDataWorld = LongDataWorld()

        val setVoxels = HashSet<Vector3ic>()

        for (i in 1..10000) {
            val x = Random.nextInt(minX..maxX)
            val y = Random.nextInt(minY..maxY)
            val z = Random.nextInt(minZ..maxZ)

            longDataWorld.setDataAt(x, y, z, SET_DATA)
            setVoxels.add(Vector3i(x, y, z))
        }

        // Assert all [setVoxels] are present in [longDataWorld]
        for (setVoxel in setVoxels) {
            Assertions.assertTrue(longDataWorld.getDataAt(setVoxel.x(), setVoxel.y(), setVoxel.z()) == SET_DATA)
        }

        for (i in 1..10000) {
            val x = Random.nextInt(minX..maxX)
            val y = Random.nextInt(minY..maxY)
            val z = Random.nextInt(minZ..maxZ)

            longDataWorld.setDataAt(x, y, z, UNSET_DATA)
            setVoxels.remove(Vector3i(x, y, z))
        }

        // Assert all [setVoxels] are present in [longDataWorld]
        for (setVoxel in setVoxels) {
            Assertions.assertTrue(longDataWorld.getDataAt(setVoxel.x(), setVoxel.y(), setVoxel.z()) == SET_DATA)
        }
    }

    @Test
    fun testSettingAndGettingSingle() {
        val SET_DATA: Long = 420
        val UNSET_DATA: Long = 0

        val longDataWorld = LongDataWorld()

        longDataWorld.setDataAt(0, 0, 0, SET_DATA)
        Assertions.assertEquals(longDataWorld.getDataAt(0, 0, 0), SET_DATA)
        longDataWorld.setDataAt(0, 0, 0, UNSET_DATA)
        Assertions.assertEquals(longDataWorld.getDataAt(0, 0, 0), UNSET_DATA)
    }
}
