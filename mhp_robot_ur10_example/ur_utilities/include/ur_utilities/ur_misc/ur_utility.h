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

#ifndef UR_UTILITY_H
#define UR_UTILITY_H

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <mhp_robot/robot_misc/robot_utility.h>
#include <ur_utilities/ur_description/ur_description.h>

namespace mhp_robot {
namespace robot_misc {

class URUtility : public RobotUtility
{
 public:
    using Ptr  = std::shared_ptr<URUtility>;
    using UPtr = std::unique_ptr<URUtility>;

    URUtility();
    URUtility(robot_description::URDescription::UPtr robot_description);
    URUtility(const std::string& urdf_param, const std::string& first_frame, const std::string& last_frame);

    RobotUtility::UPtr createUniqueInstance() const override;
    RobotUtility::Ptr createSharedInstance() const override;

    static const std::vector<std::string>& getVelocityControllerNames();
    static const std::vector<std::string>& getPositionControllerNames();

 private:
    static const std::vector<std::string> _velocity_controller_names;
    static const std::vector<std::string> _position_controller_names;
};
}  // namespace robot_misc
}  // namespace mhp_robot

#endif  // UR_UTILITY_H
