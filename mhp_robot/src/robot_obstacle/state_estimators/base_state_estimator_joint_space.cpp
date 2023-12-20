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

#include <mhp_robot/robot_obstacle/state_estimators/base_state_estimator_joint_space.h>

namespace mhp_robot {
namespace robot_obstacle {
namespace state_estimators {

BaseStateEstimatorJointSpace::BaseStateEstimatorJointSpace(int id) : _id(id)
{
    _angles               = Eigen::Vector<double, 11>::Zero();
    _angular_velocity     = Eigen::Vector<double, 11>::Zero();
    _angular_acceleration = Eigen::Vector<double, 11>::Zero();
}

void BaseStateEstimatorJointSpace::getState(Eigen::Ref<Eigen::Vector<double, 11>> angles, Eigen::Ref<Eigen::Vector<double, 11>> angular_velocity,
                                            Eigen::Ref<Eigen::Vector<double, 11>> angular_acceleration) const
{
    angles               = _angles;
    angular_velocity     = _angular_velocity;
    angular_acceleration = _angular_acceleration;
}

}  // namespace state_estimators
}  // namespace robot_obstacle
}  // namespace mhp_robot
