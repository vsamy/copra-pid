/*      File: debugUtils.h
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
#include <iostream>

// Eigen
#include <Eigen/Core>

// copra
#include <copra/PreviewSystem.h>

#ifndef _DEBUG
#define CONSTRAINT_DELETION_WARN(warn, format, ...)
#else
#define CONSTRAINT_DELETION_WARN(warn, format, ...) \
    if ((warn)) {                                   \
        fprintf(stderr, format, __VA_ARGS__);       \
    }
#endif

namespace copra {

template <class E> //see http://stackoverflow.com/questions/37181621/easy-way-of-constructing-information-message-for-throwing-stdexception-using-p
[[noreturn]] void COPRA_DLLAPI fancy_throw(std::string msg, char const* file, char const* function, std::size_t line)
{
    throw E(std::string("In file: ") + file + "(line " + std::to_string(line) + "): [In function: " + function + "]\n" + msg);
}
} // namespace copra

#define EXCEPTION(TYPE, MESSAGE) \
    fancy_throw<TYPE>(MESSAGE, __FILE__, __func__, __LINE__)
#define DOMAIN_ERROR_EXCEPTION(MESSAGE) EXCEPTION(std::domain_error, MESSAGE)
#define RUNTIME_ERROR_EXCEPTION(MESSAGE) EXCEPTION(std::runtime_error, MESSAGE)

namespace copra {

/**
 * \brief Message error where two matrices have the same number of rows.
 * \param mat1Name Name of the first checked matrix
 * \param mat2Name Name of the second checked matrix
 * \param mat1 The first checked matrix
 * \param mat2 The second checked matrix
 * \return The throwing message
 */
std::string throwMsgOnRows(const char* mat1Name, const char* mat2Name, const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2);
/**
 * \brief Message error where two matrices have the same number of rows.
 * This is the same as \see throwMsgOnRows with a longer message.
 * \param mat1Name Name of the first checked matrix
 * \param mat2Name Name of the second checked matrix
 * \param mat1 The first checked matrix
 * \param mat2 The second checked matrix
 * \return The throwing message
 */
std::string throwMsgOnRowsAskAutoSpan(const char* mat1Name, const char* mat2Name, const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2);
/**
 * \brief Message error where a matrix is square.
 * \param matName Name of the checked matrix
 * \param mat checked matrix
 * \return The throwing message
 */
std::string throwMsgOnSquareMat(const char* matName, const Eigen::MatrixXd& mat);
/**
 * \brief Message error where a matrix has a number of rows equal to dim
 * \param matName Name of the checked matrix
 * \param mat checked matrix
 * \param dim Value to be equal to
 * \return The throwing message
 */
std::string throwMsgOnRowsOnDim(const char* matName, const Eigen::MatrixXd& mat, Eigen::Index dim);
/**
 * \brief Message error where a matrix has its number of rows equal to preview system xDim
 * \param matName Name of the checked matrix
 * \param mat checked matrix
 * \param ps The preview system
 * \return The throwing message
 */
std::string throwMsgOnRowsOnPSxDim(const char* matName, const Eigen::MatrixXd& mat, const PreviewSystem* ps);
/**
 * \brief Message error where a matrix has its number of rows equal to preview system xDim or fullXDim
 * \param matName Name of the checked matrix
 * \param mat checked matrix
 * \param ps The preview system
 * \return The throwing message
 */
std::string throwMsgOnRowsOnPSXDim(const char* matName, const Eigen::MatrixXd& mat, const PreviewSystem* ps);
/**
 * \brief Message error where a matrix has its number of rows equal to preview system uDim or fullUDim
 * \param matName Name of the checked matrix
 * \param mat checked matrix
 * \param ps The preview system
 * \return The throwing message
 */
std::string throwMsgOnRowsOnPSUDim(const char* matName, const Eigen::MatrixXd& mat, const PreviewSystem* ps);
/**
 * \brief Message error where a matrix has its number of columns equal to preview system xDim or fullXDim
 * \param matName Name of the checked matrix
 * \param mat checked matrix
 * \param ps The preview system
 * \return The throwing message
 */
std::string throwMsgOnColsOnPSXDim(const char* matName, const Eigen::MatrixXd& mat, const PreviewSystem* ps);
/**
 * \brief Message error where a matrix has its number of columns equal to preview system uDim or fullUDim
 * \param matName Name of the checked matrix
 * \param mat checked matrix
 * \param ps The preview system
 * \return The throwing message
 */
std::string throwMsgOnColsOnPSUDim(const char* matName, const Eigen::MatrixXd& mat, const PreviewSystem* ps);
/**
 * \brief Message error where two matrices number of columns are equal to the preview system datas.
 * Message error where the first matrix and the second matrix are respectively equal to the preview system xDim and uDim.
 * \param mat1Name Name of the first checked matrix
 * \param mat2Name Name of the second checked matrix
 * \param mat1 The first checked matrix
 * \param mat2 The second checked matrix
 * \param ps The preview system
 * \return The throwing message
 */
std::string throwMsgOnColsOnPSxuDim(const char* mat1Name, const char* mat2Name, const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2, const PreviewSystem* ps);
/**
 * \brief Message error where two matrices number of columns are equal to the preview system datas.
 * Message error where the first matrix and the second matrix are either respectively equal to the preview system xDim and uDim
 * either respectively equal to the preview system fullXDim and fullUDim.
 * \param mat1Name Name of the first checked matrix
 * \param mat2Name Name of the second checked matrix
 * \param mat1 The first checked matrix
 * \param mat2 The second checked matrix
 * \param ps The preview system
 * \return The throwing message
 */
std::string throwMsgOnColsOnPSXUDim(const char* mat1Name, const char* mat2Name, const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2, const PreviewSystem* ps);

/**
 * \brief Message error where a matrix can not be extended to fit the new dimension.
 * \param mat The checked matrix
 * \param new_dim The new dimension asked
 * \return The throwing message
 */
std::string throwMsgOnBadNewDim(const Eigen::MatrixXd& mat, Eigen::Index new_dim);
}