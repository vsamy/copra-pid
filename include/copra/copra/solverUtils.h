
#pragma once

// stl
#include <memory>
#include <utility>

// copra
#include <copra/QuadProgSolver.h>
#include <copra/api.h>

namespace copra {

/**
 * Enum class that handles flag for selecting a qp solver.
 */
enum class SolverFlag {
    DEFAULT, /**< Default solver depending on compilation parameters (QuadProgDense -> QLD -> LSSOL -> GUROBI) */
// #if EIGEN_LSSOL
//     LSSOL, /**< Standford LSSOL solver */
// #endif
// #if EIGEN_GUROBI
//     GUROBIDense, /**< Gurobi quadratic dense solver */
// #endif
#ifdef EIGEN_QLD
    QLD, /**< Scilab QLD solver */
#endif
#ifdef EIGEN_QUADPROG
    QuadProgDense, /**< DenseMatrix version of QuadProg solver */
#endif
    // QuadProgSparse
};

/**
 * Helper function to get an unique pointer to a desired solver.
 * \param flag Flag of the solver.
 * \return An unique pointer to the desired solver.
 */
std::unique_ptr<SolverInterface> COPRA_DLLAPI solverFactory(SolverFlag flag);

/**
 * Helper function to get a desired solver.
 * This should only be used by python (unique_ptr are not bindable)
 * \param flag Flag of the solver.
 * \return The desired solver.
 */
SolverInterface* pythonSolverFactory(SolverFlag flag);

} // namespace pc