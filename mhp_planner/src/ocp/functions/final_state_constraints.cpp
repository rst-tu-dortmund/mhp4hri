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

#include <mhp_planner/ocp/functions/final_state_constraints.h>

#include <mhp_planner/numerics/matrix_utilities.h>

#include <cmath>

namespace mhp_planner {

bool TerminalBall::setWeightS(const Eigen::Ref<const Eigen::MatrixXd>& S)
{
    if (S.size() == 0) return false;
    if (!is_square(S)) return false;

    // check if we have a diagonal matrix
    if (S.isDiagonal(1e-10)) return setWeightS(S.diagonal().asDiagonal());

    _diagonal_mode               = false;
    _diagonal_mode_intentionally = false;
    _S                           = S;
    return true;
}

bool TerminalBall::setWeightS(const Eigen::DiagonalMatrix<double, -1>& S)
{
    if (S.size() == 0) return false;

    _diagonal_mode_intentionally = true;
    _diagonal_mode               = true;
    _S_diag                      = S;
    _S                           = _S_diag.toDenseMatrix();
    return true;
}

void TerminalBall::computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const
{
    assert(cost.size() == getNonIntegralStateTermDimension(k));

    if (_zero_x_ref)
    {
        if (_diagonal_mode)
            cost[0] = x_k.transpose() * _S_diag * x_k - _gamma;
        else
            cost[0] = x_k.transpose() * _S * x_k - _gamma;
    }
    else
    {
        Eigen::VectorXd xd = x_k - _x_ref->getReferenceCached(k);
        if (_diagonal_mode)
            cost[0] = xd.transpose() * _S_diag * xd - _gamma;
        else
            cost[0] = xd.transpose() * _S * xd - _gamma;
    }
}

bool TerminalBall::checkParameters(int state_dim, int control_dim, FinalStageCost::ConstPtr final_stage_cost) const
{
    bool success = true;

    if (_diagonal_mode_intentionally && _diagonal_mode)
    {
        if (_S.diagonal().size() != state_dim)
        {
            ROS_ERROR_STREAM("TerminalBall: Diagonal matrix dimension of S (" << _S_diag.diagonal().size()
                                                                              << ") does not match state vector dimension (" << state_dim
                                                                              << "); Please specify diagonal elements only.");
            success = false;
        }
    }
    else
    {
        if (_S.rows() != state_dim || _S.cols() != state_dim)
        {
            ROS_ERROR_STREAM("TerminalBall: Matrix dimension of S (" << _S.rows() << "x" << _S.cols() << ") does not match state vector dimension ("
                                                                     << state_dim << "); Please specify " << (state_dim * state_dim)
                                                                     << " elements (Row-Major).");
            success = false;
        }
    }

    return success;
}

}  // namespace mhp_planner
