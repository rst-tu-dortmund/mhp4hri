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
#include <ur_utilities/ur_description/ur_description.h>
#include <iostream>

namespace mhp_robot {
namespace robot_description {

URDescription::URDescription(const std::string& urdf_param, const std::string& first_frame, const std::string& last_frame)
    : RobotDescription(urdf_param, first_frame, last_frame)
{
}

RobotDescription::UPtr URDescription::createUniqueInstance() const
{
    return std::make_unique<URDescription>(getURDFParam(), getFirstFrame(), getLastFrame());
}

RobotDescription::Ptr URDescription::createSharedInstance() const
{
    return std::make_shared<URDescription>(getURDFParam(), getFirstFrame(), getLastFrame());
}

bool URDescription::checkParameters() const
{
    if (_urdf_model.getName() != "ur10" && _urdf_model.getName() != "ur3")
    {
        ROS_ERROR("URDescription: Not an UR urdf.");
        return false;
    }

    if (_segment_structs.size() < 9)
    {
        ROS_ERROR("URDescription: Incorrect UR Kinematic structure.");
        return false;
    }

    if (_segment_structs.size() < _link_names.size())
    {
        ROS_ERROR("URDescription: Not as many links as expected link names.");
        return false;
    }

    int n_dynamic_joints =
        std::count_if(_segment_structs.begin(), _segment_structs.end(), [](const robot_misc::Segment& s) { return !s.joint.fixed; });

    for (int i = 0; i < _link_names.size(); ++i)
    {
        if (_link_names[i].compare(_segment_structs[i].name) != 0)
        {
            ROS_ERROR("URDescription: Expected link %s at %u position", _link_names[i].c_str(), i);
            return false;
        }
    }

    if (n_dynamic_joints != 6)
    {
        ROS_ERROR("URDescription: Incorrect UR joint configuration");
        return false;
    }

    if (fabs(_segment_structs[0].joint.Rx) > 1e-5)
    {
        ROS_ERROR("URDescription: Expected no rotation around x axis on %s", _segment_structs[0].name.c_str());
        return false;
    }

    if (fabs(_segment_structs[0].joint.Ry) > 1e-5)
    {
        ROS_ERROR("URDescription: Expected no rotation around y axis on %s", _segment_structs[0].name.c_str());
        return false;
    }

    return true;
}

}  // namespace robot_description
}  // namespace mhp_robot
