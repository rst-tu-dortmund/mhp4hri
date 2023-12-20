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

#include <mhp_robot/robot_trajectory_optimization/robot_final_state_constraint.h>

namespace mhp_robot {
namespace robot_trajectory_optimization {

bool RobotFinalStateConstraint::initialize(RobotStagePreprocessor::Ptr preprocessor)
{
    if (preprocessor)
    {
        _preprocessor = preprocessor;
    }
    else
    {
        if (_use_preprocessor) ROS_WARN("RobotFinalStateConstraint: No preprocessor provided. No preprocessor is used!");
        _use_preprocessor = false;
    }

    _initialized = true;

    return true;
}

bool RobotFinalStateConstraint::isInitialized() const { return _initialized; }

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot
