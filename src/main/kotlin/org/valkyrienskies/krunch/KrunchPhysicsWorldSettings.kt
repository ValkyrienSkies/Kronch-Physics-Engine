package org.valkyrienskies.krunch

import org.valkyrienskies.krunch.SolverType.JACOBI

open class KrunchPhysicsWorldSettings(
    override var subSteps: Int = 20,
    override var iterations: Int = 2,
    override var collisionCompliance: Double = 0.0,
    override var collisionRestitutionCompliance: Double = 0.0,
    override var dynamicFrictionCompliance: Double = 0.0,
    override var speculativeContactDistance: Double = 0.05,
    override var solverType: SolverType = JACOBI,
    override var maxCollisionPoints: Int = 4,
    override val maxCollisionPointDepth: Double = 0.1
) : KrunchPhysicsWorldSettingsc
