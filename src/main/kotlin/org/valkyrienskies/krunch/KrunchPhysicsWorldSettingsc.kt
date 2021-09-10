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
}
