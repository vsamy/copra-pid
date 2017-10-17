
#include <copra/solverUtils.h>
#ifdef EIGEN_QUADPROG
#include <copra/QuadProgSolver.h>
#endif
#ifdef EIGEN_QLD
#include <copra/QLDSolver.h>
#endif
#ifdef EIGEN_GUROBI
#include <copra/GurobiSolver.h>
#endif
#ifdef EIGEN_LSSOL
#include <copra/LSSOLSolver.h>
#endif

namespace copra {

std::unique_ptr<SolverInterface> solverFactory(SolverFlag flag)
{
    switch (flag) {
#ifdef EIGEN_QUADPROG
#ifndef DEFAULT_QP_SOLVER
#define DEFAULT_QP_SOLVER
    default:
#endif
    // case SolverFlag::QuadProgSparse:
    //    return std::make_unique<QuadProgSparseSolver>();
    case SolverFlag::QuadProgDense:
        return std::unique_ptr<QuadProgDenseSolver>(new QuadProgDenseSolver());
#endif
#ifdef EIGEN_QLD
#ifndef DEFAULT_QP_SOLVER
#define DEFAULT_QP_SOLVER
    default:
#endif
    case SolverFlag::QLD:
        return std::unique_ptr<QLDSolver>(new QLDSolver());
#endif
#ifdef EIGEN_LSSOL
#ifndef DEFAULT_QP_SOLVER
#define DEFAULT_QP_SOLVER
    default:
#endif
    case SolverFlag::LSSOL:
        return std::unique_ptr<LSSOLSolver>(new LSSOLSolver());
#endif
#ifdef EIGEN_GUROBI
#ifndef DEFAULT_QP_SOLVER
#define DEFAULT_QP_SOLVER
    default:
#endif
    case SolverFlag::GUROBIDense:
        return std::unique_ptr<GUROBISolver>(new GUROBISolver());
#endif
    }
}

SolverInterface* pythonSolverFactory(SolverFlag flag)
{
    switch (flag) {
#ifdef EIGEN_QUADPROG
#ifndef DEFAULT_QP_SOLVER
#define DEFAULT_QP_SOLVER
    default:
#endif
    // case SolverFlag::QuadProgSparse:
    //    return new QuadProgSparseSolver;
    case SolverFlag::QuadProgDense:
        return new QuadProgDenseSolver();
#endif
#ifdef EIGEN_QLD
#ifndef DEFAULT_QP_SOLVER
#define DEFAULT_QP_SOLVER
    default:
#endif
    case SolverFlag::QLD:
        return new QLDSolver();
#endif
#ifdef EIGEN_LSSOL
#ifndef DEFAULT_QP_SOLVER
#define DEFAULT_QP_SOLVER
    default:
#endif
    case SolverFlag::LSSOL:
        return new LSSOLSolver();
#endif
#ifdef EIGEN_GUROBI
#ifndef DEFAULT_QP_SOLVER
#define DEFAULT_QP_SOLVER
    default:
#endif
    case SolverFlag::GUROBIDense:
        return new GUROBISolver();
#endif
    }
}

} // namespace copra