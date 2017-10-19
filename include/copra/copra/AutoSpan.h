/*      File: AutoSpan.h
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

// Eigen
#include <Eigen/Core>

namespace copra {

/**
 * \brief A class made of static helper function.
 * It can't be instantiated. 
 * This class has helper functions for automatically extending a matrix to a given dimension.
 * The result is a block diagonal matrix.
 */
class AutoSpan
{
public:
    // Delete the default constrcutor. This class should not be instantiated
    AutoSpan() = delete;

    /**
     * \brief Extend the dimension of a matrix.
     * \param mat The matrix to extend. The result is stock in this parameter.
     * \param new_dim The new dimension of the matrix
     * \param addCols Add zero columns at the end of the matrix. The number of columns added is addCols * mat.cols().
     * \throw std::domain_error if new_dim is not a multiple of mat.rows()
     * \note Does nothing if mat.rows() == new_dim
     */ 
    static void spanMatrix(Eigen::MatrixXd& mat, Eigen::Index new_dim, int addCols = 0);

    /**
     * \brief Extend the dimension of a vector.
     * \param vec The vector to extend. The result is stock in this parameter.
     * \param new_dim The new dimension of the matrix
     * \throw std::domain_error if new_dim is not a multiple of vec.rows()
     * \note Does nothing if mat.rows() == new_dim
     */ 
    static void spanVector(Eigen::VectorXd& vec, Eigen::Index new_dim);
};

} // namespace copra