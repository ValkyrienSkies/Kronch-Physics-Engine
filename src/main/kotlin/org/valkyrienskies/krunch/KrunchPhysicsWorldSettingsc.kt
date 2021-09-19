package org.valkyrienskies.krunch

/**
 * An immutable view of [KrunchPhysicsWorldSettings]
 */
interface KrunchPhysicsWorldSettingsc {
    val subSteps: Int
    val iterations: Int
    val collisionCompliance: Double
    val collisionRestitutionCompliance: Double
    val dynamicFrictionCompliance: Double

    // The distance at which a contact that isn't overlapping is included as a speculative contact
    val speculativeContactDistance: Double
    val solverType: SolverType

    // The maximum number of collision points per collision pair used during a sub-step
    val maxCollisionPoints: Int

    // The maximum depth of a collision point before we consider it too deep. If a collision point is too deep we ignore
    // it to avoid instability.
    val maxCollisionPointDepth: Double
}
