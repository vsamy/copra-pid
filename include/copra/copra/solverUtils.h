
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
    DEFAULT, /**< Default solver (QuadProgDense solver) */
#ifdef EIGEN_LSSOL_FOUND
    LSSOL, /**< Standford LSSOL solver */
#endif
#ifdef EIGEN_GUROBI_FOUND
    GUROBIDense, /**< Gurobi quadratic dense solver */
#endif
#ifdef EIGEN_QLD_FOUND
    QLD, /**< Scilab QLD solver */
#endif
    QuadProgDense, /**< DenseMatrix version of QuadProg solver */
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