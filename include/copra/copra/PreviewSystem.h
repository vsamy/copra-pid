/*      File: PreviewSystem.h
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

// eigen
#include <Eigen/Core>

// copra
#include <copra/api.h>

namespace copra {

/**
 * Structure representing all variables of a system for performing preview control.
 * Such system is defined as follow:
 * \f$x_{k+1} = Ax_{k} + Bu_{k} + d\f$.
 * After performing a recursion, this system can be represented in as:
 * \f$X = \Phi x_{0} + \Psi U + \Xi\f$.
 * \note \f$X = [x_0^T x_1^T ... x_N^T]^T\f$ and \f$U = [u_0^T u_1^T ... u_{N-1}^T]^T\f$
 * where \f$N\f$ is the dimension of the system (the number of steps).\n
 * So, \f$X\f$ dimension is \f$N+1\f$ and \f$U\f$ dimension is \f$N\f%.
 */
struct COPRA_DLLAPI PreviewSystem {
    /**
     * Default constructor.
     */
    PreviewSystem() 
    {}

    /**
     * Constructor of the class.
     * \param state The state matrix of the system.
     * \param control The control matrix of the system.
     * \param xInit The initial state.
     * \param xTraj The desired trajectory or final point.
     * \param numberOfSteps The number of step to perform.
     * \param sFlag The solver to use.
     * \throw std::domain_error if the dimension of the matrices mismatch.
     */
    PreviewSystem(const Eigen::MatrixXd& state, const Eigen::MatrixXd& control,
        const Eigen::VectorXd& bias, const Eigen::VectorXd& xInit, int numberOfSteps);

    /**
     * Create the system.
     * \param state The state matrix of the system.
     * \param control The control matrix of the system.
     * \param xInit The initial state.
     * \param xTraj The desired trajectory or final point.
     * \param numberOfSteps The number of step to perform.
     * \param sFlag The solver to use.
     * \throw std::domain_error if the dimension of the matrices mismatch.
     */
    void system(const Eigen::MatrixXd& state, const Eigen::MatrixXd& control,
        const Eigen::VectorXd& bias, const Eigen::VectorXd& xInit, int numberOfSteps);

    /**
     * \brief Update the system.
     * Fill Phi, Psi, xi in PreviewSystem
     */
    void updateSystem() noexcept;

    bool isUpdated = false; /**< State whether or not the preview system has been updated. This is done when calling the solve function of a mpc. Calling \see system or setting this to false will force a new update*/
    int nrUStep = 0; /**< The number of iteration to perform for U (it is the dimension of \f$U\f$, \f$N\f$). */
    int nrXStep = 0; /**< The number of iteration to perform for X (it is the dimension of \f$X\f$, \f$N+1\f$). */
    int xDim = 0; /**< The dimension of the state vector */
    int uDim = 0; /**< The dimension of the control vector */
    int fullXDim = 0; /**< The full dimension of the state vector (xDim*nbStep) */
    int fullUDim = 0; /**< The full dimension of the control vector (uDim*nbStep) */
    Eigen::VectorXd x0; /**< The initial state */
    Eigen::MatrixXd A; /**< The state matrix */
    Eigen::MatrixXd B; /**< The control matrix */
    Eigen::VectorXd d; /**< The bias vector */
    Eigen::MatrixXd Phi; /**< The full (after nrStep) state matrix */
    Eigen::MatrixXd Psi; /**< The full (after nrStep) control matrix */
    Eigen::VectorXd xi; /**< The full (after nrStep) bias vector */
};

} // namespace copra