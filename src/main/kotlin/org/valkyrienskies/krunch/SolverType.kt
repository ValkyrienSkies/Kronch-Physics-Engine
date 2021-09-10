package org.valkyrienskies.krunch

/**
 * The [GAUSS_SEIDEL] solver immediately updates the state of the simulation after each constraint.
 * This converges faster but introduces order dependencies.
 *
 * The [JACOBI] solver solves for all constraints, and then updates the state of the simulation once all constraints
 * are solved, applying them all simultaneously. This converges slower, but removes order dependencies.
 */
enum class SolverType {
    GAUSS_SEIDEL, JACOBI
}
