package org.valkyrienskies.krunch.collision.voxels

import org.joml.Vector3i
import org.joml.Vector3ic
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.RepeatedTest
import kotlin.random.Random
import kotlin.random.nextInt

internal class VoxelGridTest {

    @RepeatedTest(100)
    fun testSettingAndGetting() {
        val minX = Random.nextInt(-100..-1)
        val minY = Random.nextInt(-100..-1)
        val minZ = Random.nextInt(-100..-1)
        val maxX = Random.nextInt(1..100)
        val maxY = Random.nextInt(1..100)
        val maxZ = Random.nextInt(1..100)

        val voxelGrid = VoxelGrid(minX, minY, minZ, maxX, maxY, maxZ)
        val setVoxels = HashSet<Vector3ic>()

        for (i in 1..10000) {
            val x = Random.nextInt(minX..maxX)
            val y = Random.nextInt(minY..maxY)
            val z = Random.nextInt(minZ..maxZ)

            voxelGrid.setVoxelState(x, y, z, true)
            setVoxels.add(Vector3i(x, y, z))
        }

        for (setVoxel in setVoxels) {
            assertTrue(voxelGrid.getVoxelState(setVoxel.x(), setVoxel.y(), setVoxel.z()))
        }
    }
}
