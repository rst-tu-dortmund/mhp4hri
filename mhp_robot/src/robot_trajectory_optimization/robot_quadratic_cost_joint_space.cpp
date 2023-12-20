/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2023,
 *  TU Dortmund University, Institute of Control Theory and System Engineering
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
*  Authors: Maximilian Krämer
*  Maintainer(s)/Modifier(s): Heiko Renz
 *********************************************************************/

#include <mhp_robot/robot_trajectory_optimization/robot_quadratic_cost_joint_space.h>

namespace mhp_robot {
namespace robot_trajectory_optimization {

double RobotQuadraticCostJointSpace::computeStateCost(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& x_ref,
                                                      const Eigen::Ref<const Eigen::VectorXd>& s_ref)
{
    if (!_evaluate_states) return 0.0;

    double cost        = 0.0;
    Eigen::VectorXd xd = x_k - x_ref;

    cost = xd.transpose() * _Q_diag * xd;

    return cost;
}

void RobotQuadraticCostJointSpace::computeStateCostGradient(const Eigen::Ref<const Eigen::VectorXd>& x_k,
                                                            const Eigen::Ref<const Eigen::VectorXd>& x_ref,
                                                            const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::VectorXd> dx)
{
    dx = 2 * (x_k - x_ref).transpose() * _Q_diag;
}

void RobotQuadraticCostJointSpace::computeStateCostHessian(const Eigen::Ref<const Eigen::VectorXd>& x_k,
                                                           const Eigen::Ref<const Eigen::VectorXd>& x_ref,
                                                           const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::MatrixXd> dxdx)
{
    dxdx = 2 * _Q_diag;
}

int RobotQuadraticCostJointSpace::getStateCostDimension() const { return _evaluate_states ? 1 : 0; }

void RobotQuadraticCostJointSpace::setQ(const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& Q) { _Q_diag = Q; }

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot
