package org.valkyrienskies.krunch.collision.voxels

import org.joml.Vector3i
import org.joml.Vector3ic
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.RepeatedTest
import kotlin.random.Random
import kotlin.random.nextInt

internal class ListBackedPositionSetTest {

    @RepeatedTest(100)
    fun testSettingAndGetting() {
        val minX = Random.nextInt(-100000..100000)
        val minY = Random.nextInt(-100000..100000)
        val minZ = Random.nextInt(-100000..100000)
        val maxX = Random.nextInt(minX..100000)
        val maxY = Random.nextInt(minY..100000)
        val maxZ = Random.nextInt(minZ..100000)

        val positionSet = ListBackedPositionSet()

        val setVoxels = HashSet<Vector3ic>()

        for (i in 1..10000) {
            val x = Random.nextInt(minX..maxX)
            val y = Random.nextInt(minY..maxY)
            val z = Random.nextInt(minZ..maxZ)

            positionSet.addPosition(x, y, z)
            setVoxels.add(Vector3i(x, y, z))
        }

        for (i in 1..10000) {
            val x = Random.nextInt(minX..maxX)
            val y = Random.nextInt(minY..maxY)
            val z = Random.nextInt(minZ..maxZ)

            positionSet.removePosition(x, y, z)
            setVoxels.remove(Vector3i(x, y, z))
        }

        // Assert all [setVoxels] are present in [positionSet]
        for (setVoxel in setVoxels) {
            Assertions.assertTrue(positionSet.containsPosition(setVoxel.x(), setVoxel.y(), setVoxel.z()))
        }

        // Assert all [positionSet] are present in [setVoxels]
        positionSet.iterateOverAllPositionsInline { x, y, z ->
            Assertions.assertTrue(setVoxels.contains(Vector3i(x, y, z)))
        }
    }
}
