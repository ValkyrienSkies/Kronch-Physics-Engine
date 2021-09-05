package org.valkyrienskies.krunch.collision

class KrunchPhysicsWorldSettings(
    override var subSteps: Int = 20,
    override var collisionCompliance: Double = 0.0,
    override var collisionRestitutionCompliance: Double = 0.0,
    override var dynamicFrictionCompliance: Double = 0.0,
    override var restitutionCorrectionIterations: Int = 3,
    override var speculativeContactDistance: Double = 0.05
) : KrunchPhysicsWorldSettingsc
