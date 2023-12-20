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

#include <mhp_robot/robot_trajectory_optimization/robot_p_controller_joint_space.h>

namespace mhp_robot {
namespace robot_trajectory_optimization {

void RobotPControllerJointSpace::step(const Eigen::Ref<const Eigen::VectorXd>& x0, const Eigen::Ref<const Eigen::VectorXd>& x_ref,
                                      const Eigen::Ref<const Eigen::VectorXd>& u_ref, Eigen::Ref<Eigen::VectorXd> u, bool feed_forward)
{
    // control law
    u = (x_ref - x0).array() * _weights.array() + ((int)feed_forward) * u_ref.array();

    // clipping
    u = (u.array() > _max.array()).select(_max, u);
    u = (u.array() < _min.array()).select(_min, u);
}

void RobotPControllerJointSpace::setLimits(const Eigen::Ref<const Eigen::VectorXd>& min, const Eigen::Ref<const Eigen::VectorXd>& max)
{
    _min = min;
    _max = max;
}

void RobotPControllerJointSpace::setWeights(const Eigen::Ref<const Eigen::VectorXd>& weights) { _weights = weights; }

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot
