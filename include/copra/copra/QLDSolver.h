
#pragma once

// stl
#include <memory>

// eigen
#include <Eigen/Core>
#include <qp/QLD.h>

// copra
#include <copra/SolverInterface.h>
#include <copra/api.h>

namespace copra {

/**
 * QLD solver for both dense matrix.
 */

// TODO: Enable sparse matrix
class COPRA_DLLAPI QLDSolver : public SolverInterface {
public:
    /**
       * QLDSolver default constructor
       */
    QLDSolver();

    /**
     * Get information of eventual fail's solver output as define by the
     * solver documentation.
     * \return 0 The optimality conditions are satisfied.
     * \return 1 The algorithm has been stopped after too many iterations.
     * \return 2 Termination accuracy insufficient to satisfy convergence
     * criterion.
     * \return 3 Internal inconsistency of QL, division by zero.
     * \return 4 Numerical instability prevents successful termination.
     * \return 5 Length of a working array is too short.
     * \return >100 Constraints are inconsistent and fail=100+ICON, where ICON
     * denotes a constraint causing the conflict.
     */
    int SI_fail() const override;

    /**
     * Print an information on the current solver status.
     */
    void SI_inform() const override;

    /**
     * Select the print level of the solver
     * QLD can only show final results or nothing
     * \param pl 0 (hide) or else (show display)
     */
    void SI_printLevel(int pl) override;

    /**
     * Set the maximum error tolerance of the solution
     * \param tol The error tolerance
     */
    void SI_feasibilityTolerance(double tol) override;

    /**
     * Get the solver's solution.
     * \return The qp solver result.
     */
    const Eigen::VectorXd& SI_result() const override;

    /**
     * Initialize the variables of the problem to solve.
     * \see SolverInterface::SI_problem()
     * \return The qp solver result.
     */
    void SI_problem(int nrVar, int nrEq, int nrInEq) override;

    /**
     * Solve the problem.
     * \see SolverInterface::SI_solve()
     * \return The qp solver result.
     */
    bool SI_solve(const Eigen::MatrixXd& Q, const Eigen::VectorXd& c,
        const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq,
        const Eigen::MatrixXd& Aineq, const Eigen::VectorXd& bineq,
        const Eigen::VectorXd& XL, const Eigen::VectorXd& XU) override;

private:
    std::unique_ptr<Eigen::QLD> solver_;
    double eps_;
};

} // namespace pc