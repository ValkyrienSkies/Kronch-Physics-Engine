package org.valkyrienskies.krunch.collision

/**
 * An immutable view of [KrunchPhysicsWorldSettings]
 */
interface KrunchPhysicsWorldSettingsc {
    val subSteps: Int
    val collisionCompliance: Double
    val collisionRestitutionCompliance: Double
    val dynamicFrictionCompliance: Double
    val restitutionCorrectionIterations: Int

    // The distance at which a contact that isn't overlapping is included as a speculative contact
    val speculativeContactDistance: Double
}
