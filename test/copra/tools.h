
#pragma once

// Eigen
#include <Eigen/Core>

namespace {
Eigen::MatrixXd spanMatrix(const Eigen::MatrixXd& m, int size, int addCols = 0)
{
    Eigen::MatrixXd mout = Eigen::MatrixXd::Zero(m.rows() * size, m.cols() * (size + addCols));
    for (int i = 0; i < size; ++i)
        mout.block(i * m.rows(), i * m.cols(), m.rows(), m.cols()) = m;
    return mout;
}

Eigen::VectorXd spanVector(const Eigen::VectorXd& v, int size) {
    Eigen::VectorXd vout = Eigen::VectorXd::Zero(v.rows() * size);
    for (int i = 0; i < size; ++i)
        vout.segment(i * v.rows(), v.rows()) = v;
    return vout;
}
}