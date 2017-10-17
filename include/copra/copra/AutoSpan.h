
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