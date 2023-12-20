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

#include <mhp_robot/robot_trajectory_optimization/robot_state_constraint.h>

namespace mhp_robot {
namespace robot_trajectory_optimization {

RobotStateConstraint::RobotStateConstraint(int dimension) : _dim(dimension) {}

void RobotStateConstraint::compute(const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const
{
    if (!_active) return;

    cost.head(_dim) = x_k - _max.matrix();
    cost.tail(_dim) = _min.matrix() - x_k;
}

int RobotStateConstraint::getDimension() const { return _active ? 2 * _dim : 0; }

void RobotStateConstraint::setLimits(const Eigen::Ref<const Eigen::ArrayXd>& min, const Eigen::Ref<const Eigen::ArrayXd>& max)
{
    _min = min;
    _max = max;
}

void RobotStateConstraint::getLimits(Eigen::Ref<Eigen::VectorXd> min, Eigen::Ref<Eigen::VectorXd> max) const
{
    min = _min.matrix();
    max = _max.matrix();
}

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot
