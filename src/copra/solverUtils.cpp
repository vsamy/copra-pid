/*      File: solverUtils.cpp
*       This file is part of the program copra
*       Program description : This a C++ implementation of a Time Invariant Linear Model Predictive Controller (LMPC) done in C++14 with python bindings
*       Copyright (C) 2017 -  Vincent Samy (LIRMM). All Right reserved.
*
*       This software is free software: you can redistribute it and/or modify
*       it under the terms of the CeCILL-C license as published by
*       the CEA CNRS INRIA, either version 1
*       of the License, or (at your option) any later version.
*       This software is distributed in the hope that it will be useful,
*       but WITHOUT ANY WARRANTY without even the implied warranty of
*       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*       CeCILL-C License for more details.
*
*       You should have received a copy of the CeCILL-C License
*       along with this software. If not, it can be found on the official website
*       of the CeCILL licenses family (http://www.cecill.info/index.en.html).
*/
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