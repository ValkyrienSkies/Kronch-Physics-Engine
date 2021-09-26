package org.valkyrienskies.krunch

import org.valkyrienskies.krunch.SolverType.JACOBI

open class KrunchPhysicsWorldSettings(
    override var subSteps: Int = 20,
    override var iterations: Int = 2,
    /**
     * At the moment we can't set any compliance too close to 0, otherwise the solver will become unstable
     * Setting values to 1e-4 seems like a good compromise between accuracy and preventing the solver from blowing up >.<
     */
    override var collisionCompliance: Double = 1e-4,
    override var collisionRestitutionCompliance: Double = 1e-4,
    override var dynamicFrictionCompliance: Double = 1e-4,
    override var speculativeContactDistance: Double = 0.05,
    override var solverType: SolverType = JACOBI,
    override var maxCollisionPoints: Int = 4,
    override var maxCollisionPointDepth: Double = Double.MAX_VALUE
) : KrunchPhysicsWorldSettingsc
