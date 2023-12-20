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

#include <math.h>
#include <mhp_robot/robot_misc/common.h>
#include <mhp_robot/robot_misc/robot_utility.h>
#include <ros/io.h>

namespace mhp_robot {
namespace robot_misc {

RobotUtility::RobotUtility(RobotDescription::UPtr robot_description) : _robot_description(std::move(robot_description))
{
    parseRobotDescription(*_robot_description);
}

RobotUtility::RobotUtility(const std::string& urdf_param, const std::string& first_frame, const std::string& last_frame)
    : _robot_description(std::make_unique<RobotDescription>(urdf_param, first_frame, last_frame))
{
    parseRobotDescription(*_robot_description);
}

RobotUtility::UPtr RobotUtility::createUniqueInstance() const { return std::make_unique<RobotUtility>(_robot_description->createUniqueInstance()); }

RobotUtility::Ptr RobotUtility::createSharedInstance() const { return std::make_shared<RobotUtility>(_robot_description->createUniqueInstance()); }

const robot_description::RobotDescription& RobotUtility::getRobotDescription() const { return *_robot_description; }

int RobotUtility::getJointsCount(bool active) const
{
    if (active)
        return _n_active_joints;
    else
        return _n_joints;
}

const std::vector<std::string>& RobotUtility::getJointNames(bool active) const
{
    if (active)
        return _active_joint_names;
    else
        return _joint_names;
}

const std::vector<std::string>& RobotUtility::getLinkNames() const { return _link_names; }

double RobotUtility::getJointMaxLimit(int id) const { return _joint_max_limits[id]; }

double RobotUtility::getJointMinLimit(int id) const { return _joint_min_limits[id]; }

const std::vector<double>& RobotUtility::getJointMaxLimits() const { return _joint_max_limits; }

const std::vector<double>& RobotUtility::getJointMinLimits() const { return _joint_min_limits; }

const std::vector<double>& RobotUtility::getJointVelocityLimits() const { return _joint_velocity_limits; }

const std::vector<double>& RobotUtility::getJointAccelerationLimits() const { return _joint_acceleration_limits; }

const std::vector<int>& RobotUtility::getJointMapping() const { return _joint_name_mapping; }

bool RobotUtility::initJointMapping(const std::vector<std::string>& names_msg, const std::vector<std::string>& target_joint_names)
{
    int n = names_msg.size();
    int m = target_joint_names.size();

    if (!validateJointNames(target_joint_names, names_msg))
    {
        ROS_ERROR("Wrong joint names.");
        return false;
    }

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            if (target_joint_names[i].compare(names_msg[j]) == 0)
            {
                _joint_name_mapping[i] = j;
                break;
            }
        }
    }
    _initialized_mapping = true;
    return true;
}

bool RobotUtility::initJointMapping(const std::vector<std::string>& names_msg) { return initJointMapping(names_msg, _active_joint_names); }

void RobotUtility::parseJointStates(const std::vector<int>& joint_name_mapping, std::vector<double>& joint_state,
                                    const std::vector<double>& joint_state_msg, bool wrap) const
{
    if (joint_state_msg.size() < getJointsCount())
    {
        ROS_WARN_ONCE("Joint state topic contains incomplete joint state message.");
        return;
    }

    for (int i = 0; i < (int)joint_state.size() && i < (int)joint_name_mapping.size(); ++i)
    {

        joint_state[i] = wrap ? Common::wrapPItoPI(joint_state_msg[joint_name_mapping[i]]) : joint_state_msg[joint_name_mapping[i]];
    }
}

void RobotUtility::parseJointStates(std::vector<double>& joint_state, const std::vector<double>& joint_state_msg, bool wrap) const
{
    if (!_initialized_mapping)
    {
        ROS_ERROR("Joint Mapping was not initialized!");
        return;
    }

    parseJointStates(_joint_name_mapping, joint_state, joint_state_msg, wrap);
}

bool RobotUtility::isJointMappingInitialized() const { return _initialized_mapping; }

bool RobotUtility::validateJointNames(std::vector<std::string> actual_joint_names, const std::vector<std::string>& joint_names_msg) const
{
    // Check sizes
    if (joint_names_msg.size() < actual_joint_names.size()) return false;

    // Erase found joint names
    for (unsigned int i = 0; i < joint_names_msg.size(); i++)
    {
        unsigned int j;
        for (j = 0; j < actual_joint_names.size(); j++)
        {
            if (joint_names_msg[i] == actual_joint_names[j]) break;
        }
        if (joint_names_msg[i] == actual_joint_names[j])
        {
            actual_joint_names.erase(actual_joint_names.begin() + j);
        }
    }

    if (actual_joint_names.size() == 0)
    {
        return true;
    }
    else
    {
        // We have not found all joint names
        return false;
    }

    return true;
}

void RobotUtility::parseRobotDescription(const RobotDescription& robot_description)
{
    for (const Segment& segment : robot_description.getKinematicStructure())
    {
        // Active joints
        if (!segment.joint.fixed)
        {
            // Number of active joints
            ++_n_active_joints;

            // Names of active joints
            _active_joint_names.push_back(segment.joint.name);

            // Joint limits (angle, velocity, acceleration)
            _joint_max_limits.push_back(segment.joint.max);
            _joint_min_limits.push_back(segment.joint.min);
            _joint_velocity_limits.push_back(segment.joint.velocity);
            _joint_acceleration_limits.push_back(segment.joint.effort);
        }

        // Number of joints in general
        ++_n_joints;

        // Names of joints in general
        _joint_names.push_back(segment.joint.name);

        // Name of links
        _link_names.push_back(segment.name);
    }
    _joint_name_mapping = std::vector<int>(_n_active_joints, -1);
}

}  // namespace robot_misc
}  // namespace mhp_robot
