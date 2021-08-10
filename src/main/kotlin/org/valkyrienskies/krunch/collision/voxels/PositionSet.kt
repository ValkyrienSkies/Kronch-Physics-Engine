package org.valkyrienskies.krunch.collision.voxels

/**
 * A set of positions.
 */
interface PositionSet {
    /**
     * Returns true if this [PositionSet] changed, false otherwise.
     */
    fun addPosition(x: Int, y: Int, z: Int): Boolean

    /**
     * Returns true if this [PositionSet] changed, false otherwise.
     */
    fun removePosition(x: Int, y: Int, z: Int): Boolean

    /**
     * Returns true if this [PositionSet] contains the position at [x], [y], [z].
     */
    fun containsPosition(x: Int, y: Int, z: Int): Boolean

    /**
     * Run [function] for every position in this [PositionSet].
     */
    fun iterateOverAllPositions(function: (x: Int, y: Int, z: Int) -> Unit)
}
