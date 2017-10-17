
#define BOOST_TEST_MODULE TestSolvers

// stl
#include <iostream>
#include <numeric>

// boost
#include <boost/test/unit_test.hpp>

// eigen
#include <Eigen/Core>

// copra
#include <copra/solverUtils.h>

// Tests problems
#include "systems.h"

//TODO: better checks
#ifdef EIGEN_QUADPROG
BOOST_FIXTURE_TEST_CASE(QuadProgTest, Problem)
{
    auto qp = solverFactory(copra::SolverFlag::QuadProgDense);

    qp->SI_problem(nrvars, nreqs, nrineqs);
    BOOST_REQUIRE(qp->SI_solve(Q, c, Aeq, beq, Aineq, bineq, XL, XU));

    BOOST_REQUIRE_EQUAL(qp->SI_fail(), 0);
}
#endif

#ifdef EIGEN_QLD
BOOST_FIXTURE_TEST_CASE(QLDTest, Problem)
{
    auto qp = solverFactory(copra::SolverFlag::QLD);

    qp->SI_problem(nrvars, nreqs, nrineqs);
    BOOST_REQUIRE(qp->SI_solve(Q, c, Aeq, beq, Aineq, bineq, XL, XU));

    BOOST_REQUIRE_EQUAL(qp->SI_fail(), 0);
}
#endif

#ifdef EIGEN_LSSOL
BOOST_FIXTURE_TEST_CASE(LSSOLTest, Problem)
{
    auto qp = solverFactory(copra::SolverFlag::LSSOL);

    qp->SI_problem(nrvars, nreqs, nrineqs);
    BOOST_REQUIRE(qp->SI_solve(Q, c, Aeq, beq, Aineq, bineq, XL, XU));

    BOOST_REQUIRE_EQUAL(qp->SI_fail(), 0);
}
#endif

#ifdef EIGEN_GUROBI
BOOST_FIXTURE_TEST_CASE(GUROBITest, Problem)
{
    auto qp = solverFactory(copra::SolverFlag::Gurobi);

    qp->SI_problem(nrvars, nreqs, nrineqs);
    BOOST_REQUIRE(qp->SI_solve(Q, c, Aeq, beq, Aineq, bineq, XL, XU));

    BOOST_REQUIRE_EQUAL(qp->SI_fail(), 0);
}
#endif