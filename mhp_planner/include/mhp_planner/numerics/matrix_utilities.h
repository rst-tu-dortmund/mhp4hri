/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020,
 *  TU Dortmund University, Institute of Control Theory and System Enginnering
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Authors: Christoph Rösmann
 *  Modifier(s)/Maintainer(s): Maximilian Krämer, Heiko Renz
 *********************************************************************/

#ifndef SRC_NUMERICS_INCLUDE_MHP_PLANNER_NUMERICS_MATRIX_UTILITIES_H_
#define SRC_NUMERICS_INCLUDE_MHP_PLANNER_NUMERICS_MATRIX_UTILITIES_H_

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <type_traits>

namespace mhp_planner {

/**
 * @brief Determine if a given matrix is square
 * @ingroup numerics
 *
 * @param matrix generic matrix type to be tested
 * @returns true if square, false otherwise.
 **/
template <typename Derived>
inline bool is_square(const Eigen::MatrixBase<Derived>& matrix)
{
    return matrix.rows() == matrix.cols();
}

/**
 * @brief Determine if a given number of elements defines a square matrix
 * @ingroup numerics
 *
 * @param numel number of elements to be checked for square matrix
 * @returns true if square, false otherwise.
 **/
inline bool is_square(int numel)
{
    double q = std::sqrt(numel);
    if (q * q != numel) return false;
    return true;
}

}  // namespace mhp_planner

#endif  // SRC_NUMERICS_INCLUDE_MHP_PLANNER_NUMERICS_MATRIX_UTILITIES_H_
