package org.valkyrienskies.krunch.solver

import org.joml.Quaterniond
import org.joml.Vector3d
import org.junit.jupiter.api.Test
import org.valkyrienskies.krunch.Body
import org.valkyrienskies.krunch.KrunchPhysicsWorldSettings
import org.valkyrienskies.krunch.Pose
import org.valkyrienskies.krunch.SolverType.JACOBI
import org.valkyrienskies.krunch.constraints.CollisionConstraint
import org.valkyrienskies.krunch.constraints.RestitutionConstraint

internal class JacobiSolverTest {

    @Test
    fun testRestitutionCorrectionConvergence() {
        val settings = KrunchPhysicsWorldSettings()
        settings.iterations = 2
        settings.solverType = JACOBI

        val dt = 8.333333333333333E-4

        val body0 = Body(Pose())
        body0.isStatic = true
        body0.coefficientOfRestitution = 0.5
        body0.staticFrictionCoefficient = 1.0
        body0.dynamicFrictionCoefficient = 0.4
        body0.setBox(Vector3d(1.0, 1.0, 1.0))

        val body1 = Body(
            Pose(
                Vector3d(0.0, 2.000809094340756, 0.0),
                Quaterniond(-7.268702886115656E-4, -3.0762739520651943E-7, 8.927832947852375E-4, 0.9999993372985192)
            )
        )
        body1.prevPose.set(Pose(Vector3d(0.0, 2.000583333342764, 0.0), Quaterniond()))
        body1.vel.set(0.0, 0.2709131975905166, 0.0)
        body1.omega.set(-1.7444886926677576, -7.383057484956466E-4, 2.1426799074845704)
        body1.prevVel.set(0.0, -4.474999999957853, 0.0)
        body1.prevOmega.set(0.0, 0.0, 0.0)

        body1.coefficientOfRestitution = 0.5
        body1.staticFrictionCoefficient = 1.0
        body1.dynamicFrictionCoefficient = 0.4
        body1.setBox(Vector3d(1.0, 1.0, 1.0))

        val collisionConstraints = ArrayList<CollisionConstraint>()
        run {
            val collisionConstraint0 = CollisionConstraint(
                body0, Vector3d(0.25, 1.5, 0.25), body1, Vector3d(0.25, -0.5, 0.25), Vector3d(0.0, 1.0, 0.0), 0.0
            )
            collisionConstraint0.lambda = -0.0017976190422100812
            collisionConstraint0.prevLambda = -0.0017976190422100812
            collisionConstraints.add(collisionConstraint0)

            val collisionConstraint1 = CollisionConstraint(
                body0, Vector3d(0.25, 1.5, -0.25), body1, Vector3d(0.25, -0.5, -0.25), Vector3d(0.0, 1.0, 0.0), 0.0
            )
            collisionConstraint1.lambda = -7.683212415628233E-4
            collisionConstraint1.prevLambda = -7.683212415628233E-4
            collisionConstraints.add(collisionConstraint1)

            val collisionConstraint2 = CollisionConstraint(
                body0, Vector3d(-0.25, 1.5, 0.25), body1, Vector3d(-0.25, -0.5, 0.25), Vector3d(0.0, 1.0, 0.0), 0.0
            )
            collisionConstraint2.lambda = -6.594752412773617E-4
            collisionConstraint2.prevLambda = -6.594752412773617E-4
            collisionConstraints.add(collisionConstraint2)

            val collisionConstraint3 = CollisionConstraint(
                body0, Vector3d(-0.25, 1.5, -0.25), body1, Vector3d(-0.25, -0.5, -0.25), Vector3d(0.0, 1.0, 0.0), 0.0
            )
            collisionConstraint3.lambda = -7.295121395736913E-4
            collisionConstraint3.prevLambda = -7.295121395736913E-4
            collisionConstraints.add(collisionConstraint3)
        }

        val restitutionConstraints = ArrayList<RestitutionConstraint>()
        collisionConstraints.forEach { collisionConstraint ->
            val restitutionConstraint = RestitutionConstraint(
                collisionConstraint.body0, collisionConstraint.body0ContactPosInBody0Coordinates,
                collisionConstraint.body1,
                collisionConstraint.body1ContactPosInBody1Coordinates,
                collisionConstraint.contactNormalInGlobalCoordinates, 0.0,
                collisionConstraint
            )
            restitutionConstraints.add(restitutionConstraint)
        }

        val solver = JacobiSolver()

        solver.solveVelocityConstraints(listOf(body0, body1), restitutionConstraints, settings.iterations, dt)

        // Currently output should be:
        // body1.vel = ( 0.000E+0 -2.970E+2  0.000E+0 )
        // body1.omega = ( 1.193E+0 -7.383E-4 -1.466E+0)
        val i = 1

        // TODO: Test convergence of Jacobi at the end
    }
}
