/*      File: constraints.cpp
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
#include <copra/constraints.h>

// copra
#include <copra/AutoSpan.h>
#include <copra/PreviewSystem.h>

// stl
#include <sstream>

namespace copra {

/*************************************************************************************************
 *                                         Constraint                                            *
 *************************************************************************************************/

Constraint::Constraint(std::string&& name)
    : name_(std::move(name))
    , nrConstr_(0)
    , fullSizeEntry_(false)
    , hasBeenInitialized_(false)
{
}

/*************************************************************************************************
 *                           Equality or Inequality Constraint                                   *
 *************************************************************************************************/

EqIneqConstraint::EqIneqConstraint(const std::string& constrQualifier, bool isInequalityConstraint)
    : Constraint(constrQualifier + (isInequalityConstraint ? " inequality constraint" : " equality constraint"))
    , A_()
    , b_()
    , isIneq_(isInequalityConstraint)
{
}

/*************************************************************************************************
 *                                  Trajectory Constraint                                        *
 *************************************************************************************************/

void TrajectoryConstraint::autoSpan()
{
    auto max_dim = std::max(E_.rows(), f_.rows());
    AutoSpan::spanMatrix(E_, max_dim);
    AutoSpan::spanVector(f_, max_dim);
}

void TrajectoryConstraint::initializeConstraint(const PreviewSystem& ps)
{
    if (E_.rows() != f_.rows())
        DOMAIN_ERROR_EXCEPTION(throwMsgOnRowsAskAutoSpan("E", "f", E_, f_));

    if (E_.cols() == ps.xDim) {
        nrConstr_ = static_cast<int>(E_.rows()) * ps.nrXStep;

    } else if (E_.cols() == ps.fullXDim) {
        fullSizeEntry_ = true;
        nrConstr_ = static_cast<int>(E_.rows());
    } else {
        DOMAIN_ERROR_EXCEPTION(throwMsgOnColsOnPSXDim("E", E_, &ps));
    }

    A_.resize(nrConstr_, ps.fullUDim);
    b_.resize(nrConstr_);
}

void TrajectoryConstraint::update(const PreviewSystem& ps)
{
    if (fullSizeEntry_) {
        A_.noalias() = E_ * ps.Psi;
        b_.noalias() = f_ - E_ * (ps.Phi * ps.x0 + ps.xi);
    } else {
        auto nrLines = static_cast<int>(E_.rows());
        for (int i = 0; i < ps.nrXStep; ++i) {
            A_.block(i * nrLines, 0, nrLines, ps.fullUDim).noalias() = E_ * ps.Psi.block(i * ps.xDim, 0, ps.xDim, ps.fullUDim);
            b_.segment(i * nrLines, nrLines).noalias() = f_ - E_ * (ps.Phi.block(i * ps.xDim, 0, ps.xDim, ps.xDim) * ps.x0 + ps.xi.segment(i * ps.xDim, ps.xDim));
        }
    }
}

ConstraintFlag TrajectoryConstraint::constraintType() const noexcept
{
    if (isIneq_)
        return ConstraintFlag::InequalityConstraint;
    else
        return ConstraintFlag::EqualityConstraint;
}

/*************************************************************************************************
 *                                    Control Constraint                                         *
 *************************************************************************************************/

void ControlConstraint::autoSpan()
{
    auto max_dim = std::max(G_.rows(), f_.rows());
    AutoSpan::spanMatrix(G_, max_dim);
    AutoSpan::spanVector(f_, max_dim);
}

void ControlConstraint::initializeConstraint(const PreviewSystem& ps)
{
    if (hasBeenInitialized_)
        RUNTIME_ERROR_EXCEPTION("You have initialized a ControlConstraint twice. As move semantics are used, you can't do so.");

    if (G_.rows() != f_.rows())
        DOMAIN_ERROR_EXCEPTION(throwMsgOnRowsAskAutoSpan("G", "f", G_, f_));

    if (G_.cols() == ps.uDim) {
        nrConstr_ = static_cast<int>(G_.rows()) * ps.nrUStep;
        A_.resize(nrConstr_, ps.fullUDim);
        b_.resize(nrConstr_);
        A_.setZero();
    } else if (G_.cols() == ps.fullUDim) {
        fullSizeEntry_ = true;
        nrConstr_ = static_cast<int>(G_.rows());
        A_ = std::move(G_);
        b_ = std::move(f_);
    } else {
        DOMAIN_ERROR_EXCEPTION(throwMsgOnColsOnPSUDim("G", G_, &ps));
    }

    hasBeenInitialized_ = true;
}

void ControlConstraint::update(const PreviewSystem& ps)
{
    if (!fullSizeEntry_) {
        auto nrLines = static_cast<int>(G_.rows());
        for (int i = 0; i < ps.nrUStep; ++i) {
            A_.block(i * nrLines, i * ps.uDim, nrLines, ps.uDim) = G_;
            b_.segment(i * nrLines, nrLines) = f_;
        }
    }
}

ConstraintFlag ControlConstraint::constraintType() const noexcept
{
    if (isIneq_)
        return ConstraintFlag::InequalityConstraint;
    else
        return ConstraintFlag::EqualityConstraint;
}

/*************************************************************************************************
 *                                      Mixed Constraint                                         *
 *************************************************************************************************/

void MixedConstraint::autoSpan()
{
    auto max_dim = std::max(f_.rows(), std::max(E_.rows(), G_.rows()));
    AutoSpan::spanMatrix(E_, max_dim, 1); // This is tricky. Has X and U are not the same dimensions, we need to handle it.
    AutoSpan::spanMatrix(G_, max_dim);
    AutoSpan::spanVector(f_, max_dim);
}

void MixedConstraint::initializeConstraint(const PreviewSystem& ps)
{
    if (E_.rows() != f_.rows())
        DOMAIN_ERROR_EXCEPTION(throwMsgOnRowsAskAutoSpan("E", "f", E_, f_));
    if (G_.rows() != f_.rows())
        DOMAIN_ERROR_EXCEPTION(throwMsgOnRowsAskAutoSpan("G", "f", G_, f_));

    if (E_.cols() == ps.xDim && G_.cols() == ps.uDim) {
        nrConstr_ = static_cast<int>(E_.rows()) * ps.nrUStep;
    } else if (E_.cols() == ps.fullXDim && G_.cols() == ps.fullUDim) {
        fullSizeEntry_ = true;
        nrConstr_ = static_cast<int>(E_.rows());
    } else {
        DOMAIN_ERROR_EXCEPTION(throwMsgOnColsOnPSXUDim("E", "G", E_, G_, &ps));
    }

    A_.resize(nrConstr_, ps.fullUDim);
    b_.resize(nrConstr_);
    A_.setZero();
}

void MixedConstraint::update(const PreviewSystem& ps)
{
    if (fullSizeEntry_) {
        A_.noalias() = E_ * ps.Psi + G_;
        b_.noalias() = f_ - E_ * (ps.Phi * ps.x0 + ps.xi);
    } else {
        auto nrLines = static_cast<int>(E_.rows());
        auto uDim = ps.uDim;
        auto xDim = ps.xDim;
        A_.block(0, 0, nrLines, uDim) = G_;
        b_.head(nrLines) = f_ - E_ * ps.x0;
        for (int i = 1; i < ps.nrUStep; ++i) {
            A_.block(i * nrLines, 0, nrLines, uDim) = E_ * ps.Psi.block(i * xDim, 0, xDim, uDim);
            for (int j = 1; j <= i; ++j)
                A_.block(i * nrLines, j * uDim, nrLines, uDim) = A_.block((i - 1) * nrLines, (j - 1) * uDim, nrLines, uDim);

            b_.segment(i * nrLines, nrLines) = f_ - E_ * (ps.Phi.block(i * xDim, 0, xDim, xDim) * ps.x0 + ps.xi.segment(i * xDim, xDim));
        }
    }
}

ConstraintFlag MixedConstraint::constraintType() const noexcept
{
    if (isIneq_)
        return ConstraintFlag::InequalityConstraint;
    else
        return ConstraintFlag::EqualityConstraint;
}

/*************************************************************************************************
 *                               Trajectory Bound Constraint                                     *
 *************************************************************************************************/

void TrajectoryBoundConstraint::autoSpan()
{
    auto max_dim = std::max(lower_.rows(), upper_.rows());
    AutoSpan::spanVector(lower_, max_dim);
    AutoSpan::spanVector(upper_, max_dim);
    if (lower_.rows() != max_dim) {
        lowerLines_.clear();
        for (auto line = 0; line < lower_.rows(); ++line) {
            if (lower_(line) != -std::numeric_limits<double>::infinity())
                lowerLines_.push_back(line);
        }
    }
    if (upper_.rows() != max_dim) {
        upperLines_.clear();
        for (auto line = 0; line < upper_.rows(); ++line) {
            if (upper_(line) != std::numeric_limits<double>::infinity())
                upperLines_.push_back(line);
        }
    }
}

void TrajectoryBoundConstraint::initializeConstraint(const PreviewSystem& ps)
{
    if (lower_.rows() != upper_.rows())
        DOMAIN_ERROR_EXCEPTION(throwMsgOnRowsAskAutoSpan("lower", "upper", lower_, upper_));

    if (lower_.rows() == ps.xDim) {
        nrConstr_ = static_cast<int>((lowerLines_.size() + upperLines_.size())) * ps.nrXStep;
    } else if (lower_.rows() == ps.fullXDim) {
        nrConstr_ = static_cast<int>((lowerLines_.size() + upperLines_.size()));
        fullSizeEntry_ = true;
    } else {
        DOMAIN_ERROR_EXCEPTION(throwMsgOnColsOnPSXDim("lower", lower_, &ps));
    }

    A_.resize(nrConstr_, ps.fullUDim);
    b_.resize(nrConstr_);
}

void TrajectoryBoundConstraint::update(const PreviewSystem& ps)
{
    int nrLines = 0;
    Eigen::VectorXd delta = ps.Phi * ps.x0 + ps.xi;
    for (auto step = 0; step < ps.nrXStep; ++step) {
        for (auto line : lowerLines_) {
            A_.row(nrLines) = ps.Psi.row(line + ps.xDim * step);
            b_(nrLines) = lower_(line) - delta(line + ps.xDim * step);
            ++nrLines;
        }
        if (fullSizeEntry_)
            break;
    }

    for (auto step = 0; step < ps.nrXStep; ++step) {
        for (auto line : upperLines_) {
            A_.row(nrLines) = ps.Psi.row(line + ps.xDim * step);
            b_(nrLines) = upper_(line) - delta(line + ps.xDim * step);
            ++nrLines;
        }
        if (fullSizeEntry_)
            break;
    }
}

ConstraintFlag TrajectoryBoundConstraint::constraintType() const noexcept
{
    return ConstraintFlag::InequalityConstraint;
}

/*************************************************************************************************
 *                                 Control Bound Constraint                                      *
 *************************************************************************************************/

void ControlBoundConstraint::autoSpan()
{
    auto max_dim = std::max(lower_.rows(), upper_.rows());
    AutoSpan::spanVector(lower_, max_dim);
    AutoSpan::spanVector(upper_, max_dim);
}

void ControlBoundConstraint::initializeConstraint(const PreviewSystem& ps)
{
    if (hasBeenInitialized_)
        RUNTIME_ERROR_EXCEPTION("You have initialized a ControlBoundConstraint twice. As move semantics are used, you can't do so.");

    if (lower_.rows() != upper_.rows())
        DOMAIN_ERROR_EXCEPTION(throwMsgOnRowsAskAutoSpan("lower", "upper", lower_, upper_));

    if (lower_.rows() == ps.uDim) {
        nrConstr_ = ps.fullUDim;
        lb_.resize(nrConstr_);
        ub_.resize(nrConstr_);
    } else if (lower_.rows() == ps.fullUDim) {
        fullSizeEntry_ = true;
        nrConstr_ = static_cast<int>(lower_.rows());
        lb_ = std::move(lower_);
        ub_ = std::move(upper_);
    } else {
        DOMAIN_ERROR_EXCEPTION(throwMsgOnColsOnPSUDim("lower", lower_, &ps));
    }

    hasBeenInitialized_ = true;
}

void ControlBoundConstraint::update(const PreviewSystem& ps)
{
    if (!fullSizeEntry_) {
        for (int i = 0; i < ps.nrUStep; ++i) {
            ub_.segment(i * ps.uDim, ps.uDim) = upper_;
            lb_.segment(i * ps.uDim, ps.uDim) = lower_;
        }
    }
}

ConstraintFlag ControlBoundConstraint::constraintType() const noexcept
{
    return ConstraintFlag::BoundConstraint;
}

} // namespace copra