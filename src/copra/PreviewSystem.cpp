/*      File: PreviewSystem.cpp
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
// header
#include <copra/PreviewSystem.h>

//copra
#include <copra/debugUtils.h>

namespace copra {

PreviewSystem::PreviewSystem(const Eigen::MatrixXd& state, const Eigen::MatrixXd& control,
    const Eigen::VectorXd& bias, const Eigen::VectorXd& xInit, int numberOfSteps)
{
    system(state, control, bias, xInit, numberOfSteps);
}

void PreviewSystem::system(const Eigen::MatrixXd& state, const Eigen::MatrixXd& control,
    const Eigen::VectorXd& bias, const Eigen::VectorXd& xInit, int numberOfSteps)
{
    if (xInit.rows() != state.rows())
        DOMAIN_ERROR_EXCEPTION(throwMsgOnRows("xInit", "state", xInit, state));
    if (state.rows() != state.cols())
        DOMAIN_ERROR_EXCEPTION(throwMsgOnSquareMat("state", state));
    if (xInit.rows() != control.rows())
        DOMAIN_ERROR_EXCEPTION(throwMsgOnRows("xInit", "control", xInit, control));
    if (xInit.rows() != bias.rows())
        DOMAIN_ERROR_EXCEPTION(throwMsgOnRows("xInit", "state", xInit, bias));
    if (numberOfSteps <= 0) // This should desappear later
        DOMAIN_ERROR_EXCEPTION("The number of step sould be a positive number! ");

    isUpdated = false;
    nrUStep = numberOfSteps;
    nrXStep = numberOfSteps + 1;
    xDim = static_cast<int>(state.cols());
    uDim = static_cast<int>(control.cols());
    fullXDim = xDim * nrXStep;
    fullUDim = uDim * nrUStep;

    x0 = xInit;
    A = state;
    B = control;
    d = bias;
    Phi.resize(fullXDim, xDim);
    Psi.resize(fullXDim, fullUDim);
    xi.resize(fullXDim);

    Phi.setZero();
    Phi.block(0, 0, xDim, xDim) = Eigen::MatrixXd::Identity(xDim, xDim);
    Psi.setZero();
    xi.setZero();
}

void PreviewSystem::updateSystem() noexcept
{
    Phi.block(xDim, 0, xDim, xDim) = A;
    Psi.block(xDim, 0, xDim, uDim) = B;
    xi.segment(xDim, xDim) = d;

    for (auto i = 2; i < nrXStep; ++i) {
        Phi.block(i * xDim, 0, xDim, xDim).noalias() = A * Phi.block((i - 1) * xDim, 0, xDim, xDim);
        Psi.block(i * xDim, 0, xDim, uDim).noalias() = A * Psi.block((i - 1) * xDim, 0, xDim, uDim);
        for (auto j = 1; j < i; ++j)
            Psi.block(i * xDim, j * uDim, xDim, uDim) = Psi.block((i - 1) * xDim, (j - 1) * uDim, xDim, uDim);

        xi.segment(i * xDim, xDim).noalias() = A * xi.segment((i - 1) * xDim, xDim) + d;
    }

    isUpdated = true;
}

} // namespace copra