package org.valkyrienskies.krunch

import org.valkyrienskies.krunch.SolverType.GAUSS_SEIDEL

open class KrunchPhysicsWorldSettings(
    override var subSteps: Int = 20,
    override var iterations: Int = 20,
    override var collisionCompliance: Double = 0.0,
    override var collisionRestitutionCompliance: Double = 0.0,
    override var dynamicFrictionCompliance: Double = 0.0,
    override var speculativeContactDistance: Double = 0.05,
    override var solverType: SolverType = GAUSS_SEIDEL
) : KrunchPhysicsWorldSettingsc
