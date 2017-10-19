/*      File: solverUtils.h
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