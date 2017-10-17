// This file is part of copra.

// copra is free software: you can redistribute it and/or
// modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// copra is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with copra.  If not, see
// <http://www.gnu.org/licenses/>.

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