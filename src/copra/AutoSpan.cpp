/*      File: AutoSpan.cpp
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
#include <copra/AutoSpan.h>

// copra
#include <copra/debugUtils.h>

namespace copra {

void AutoSpan::spanMatrix(Eigen::MatrixXd& mat, Eigen::Index new_dim, int addCols)
{
    auto matRows = mat.rows();
    if (new_dim == matRows)
        return;

    auto matCols = mat.cols();
    auto tmp = mat;
    auto nrStep = new_dim / matRows;
    if (nrStep * matRows != new_dim)
        DOMAIN_ERROR_EXCEPTION(throwMsgOnBadNewDim(mat, new_dim));

    mat = Eigen::MatrixXd::Zero(new_dim, matCols * (nrStep + addCols));
    for (auto i = 0; i < nrStep; ++i)
        mat.block(i * matRows, i * matCols, matRows, matCols) = tmp;
}

void AutoSpan::spanVector(Eigen::VectorXd& vec, Eigen::Index new_dim)
{
    auto vecRows = vec.rows();
    if (new_dim == vecRows)
        return;

    auto nrStep = new_dim / vecRows;
    auto tmp = vec;
    if (nrStep * vecRows != new_dim)
        DOMAIN_ERROR_EXCEPTION(throwMsgOnBadNewDim(vec, new_dim));

    vec.conservativeResize(new_dim);
    for (auto i = 1; i < nrStep; ++i)
        vec.segment(i * vecRows, vecRows) = tmp;
}

} // namespace copra