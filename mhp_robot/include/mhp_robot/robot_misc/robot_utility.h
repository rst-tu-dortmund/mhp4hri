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

#ifndef ROBOT_UTILITY_H
#define ROBOT_UTILITY_H

#include <mhp_robot/robot_description/robot_description.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <utility>
#include <vector>

namespace mhp_robot {
namespace robot_misc {

class RobotUtility
{
 public:
    using Ptr  = std::shared_ptr<RobotUtility>;
    using UPtr = std::unique_ptr<RobotUtility>;

    RobotUtility(robot_description::RobotDescription::UPtr robot_description);
    RobotUtility(const std::string& urdf_param, const std::string& first_frame, const std::string& last_frame);

    RobotUtility(const RobotUtility&)            = delete;
    RobotUtility(RobotUtility&&)                 = default;
    RobotUtility& operator=(const RobotUtility&) = delete;
    RobotUtility& operator=(RobotUtility&&)      = default;
    virtual ~RobotUtility() {}

    virtual UPtr createUniqueInstance() const;
    virtual Ptr createSharedInstance() const;

    const robot_description::RobotDescription& getRobotDescription() const;

    int getJointsCount(bool active = true) const;

    const std::vector<std::string>& getJointNames(bool active = true) const;

    const std::vector<std::string>& getLinkNames() const;

    double getJointMaxLimit(int id) const;

    double getJointMinLimit(int id) const;

    const std::vector<double>& getJointMaxLimits() const;

    const std::vector<double>& getJointMinLimits() const;

    const std::vector<double>& getJointVelocityLimits() const;

    const std::vector<double>& getJointAccelerationLimits() const;

    const std::vector<int>& getJointMapping() const;

    bool initJointMapping(const std::vector<std::string>& names_msg, const std::vector<std::string>& target_joint_names);
    bool initJointMapping(const std::vector<std::string>& names_msg);

    void parseJointStates(const std::vector<int>& joint_name_mapping, std::vector<double>& joint_state, const std::vector<double>& joint_state_msg,
                          bool wrap = false) const;
    void parseJointStates(std::vector<double>& joint_state, const std::vector<double>& joint_state_msg, bool wrap = false) const;

    bool isJointMappingInitialized() const;

    bool validateJointNames(std::vector<std::string> actual_joint_names, const std::vector<std::string>& joint_names_msg) const;

 protected:
    using RobotDescription = robot_description::RobotDescription;

    RobotDescription::UPtr _robot_description;

 private:
    std::vector<int> _joint_name_mapping;
    std::vector<std::string> _joint_names;
    std::vector<std::string> _active_joint_names;
    std::vector<std::string> _link_names;
    std::vector<double> _joint_max_limits;
    std::vector<double> _joint_min_limits;
    std::vector<double> _joint_velocity_limits;
    std::vector<double> _joint_acceleration_limits;

    bool _initialized_mapping = false;
    int _n_active_joints      = 0;
    int _n_joints             = 0;

    void parseRobotDescription(const RobotDescription& robot_description);
};

}  // namespace robot_misc
}  // namespace mhp_robot

#endif  // ROBOT_UTILITY_H
