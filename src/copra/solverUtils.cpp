
#include <copra/solverUtils.h>

namespace copra {

std::unique_ptr<SolverInterface> solverFactory(SolverFlag flag)
{
    switch (flag) {
#ifdef EIGEN_LSSOL_FOUND
    case SolverFlag::LSSOL:
        return std::unique_ptr<LSSOLSolver>(new LSSOLSolver());
#endif
#ifdef EIGEN_GUROBI_FOUND
    case SolverFlag::GUROBIDense:
        return std::unique_ptr<GUROBISolver>(new GUROBISolver());
#endif
#ifdef EIGEN_QLD_FOUND
    case SolverFlag::QLD:
        return std::unique_ptr<QLDSolver>(new QLDSolver());
#endif
    // case SolverFlag::QuadProgSparse:
    //    return std::make_unique<QuadProgSparseSolver>();
    case SolverFlag::QuadProgDense:
    default:
        return std::unique_ptr<QuadProgDenseSolver>(new QuadProgDenseSolver());
    }
}

SolverInterface* pythonSolverFactory(SolverFlag flag)
{
    switch (flag) {
#ifdef EIGEN_LSSOL_FOUND
    case SolverFlag::LSSOL:
        return new LSSOLSolver;
#endif
#ifdef EIGEN_GUROBI_FOUND
    case SolverFlag::GUROBIDense:
        return new GUROBISolver;
#endif
#ifdef EIGEN_QLD_FOUND
    case SolverFlag::QLD:
        return new QLDSolver;
#endif
    case SolverFlag::QuadProgDense:
    default:
        return new QuadProgDenseSolver;
    }
}

} // namespace copra