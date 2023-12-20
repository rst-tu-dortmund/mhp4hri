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

#include <ur_utilities/ur_misc/ur_utility.h>

namespace mhp_robot {
namespace robot_misc {

URUtility::URUtility(robot_description::URDescription::UPtr robot_description) : RobotUtility(std::move(robot_description)) {}

URUtility::URUtility(const std::string& urdf_param, const std::string& first_frame, const std::string& last_frame)
    : RobotUtility(std::make_unique<robot_description::URDescription>(urdf_param, first_frame, last_frame))
{
}

URUtility::URUtility() : RobotUtility(std::make_unique<robot_description::URDescription>("/robot_description", "world", "ee_link")) {}

RobotUtility::UPtr URUtility::createUniqueInstance() const
{
    return std::make_unique<URUtility>(_robot_description->getURDFParam(), _robot_description->getFirstFrame(), _robot_description->getLastFrame());
}

RobotUtility::Ptr URUtility::createSharedInstance() const
{
    return std::make_shared<URUtility>(_robot_description->getURDFParam(), _robot_description->getFirstFrame(), _robot_description->getLastFrame());
}

const std::vector<std::string>& URUtility::getVelocityControllerNames() { return _velocity_controller_names; }

const std::vector<std::string>& URUtility::getPositionControllerNames() { return _position_controller_names; }

const std::vector<std::string> URUtility::_position_controller_names = {"vel_based_pos_traj_controller"};

const std::vector<std::string> URUtility::_velocity_controller_names = {"joint0_velocity_controller", "joint1_velocity_controller",
                                                                        "joint2_velocity_controller", "joint3_velocity_controller",
                                                                        "joint4_velocity_controller", "joint5_velocity_controller"};

}  // namespace robot_misc
}  // namespace mhp_robot
