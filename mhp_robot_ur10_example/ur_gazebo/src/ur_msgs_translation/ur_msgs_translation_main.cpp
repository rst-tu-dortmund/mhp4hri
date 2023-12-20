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

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ur_utilities/ur_misc/ur_utility.h>
#include <string>
#include <vector>

mhp_robot::robot_misc::URUtility::Ptr _ur_utility;
std::vector<ros::Publisher> _pub_vel;
bool _first_msg = true;

void velocityCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    if (msg->points.size() < 1)
    {
        ROS_ERROR_STREAM(
            "Error in converter node: No velocities given! Message is "
            "not converted!"
            << std::endl);
        return;
    }

    std::vector<double> velocity             = msg->points[0].velocities;
    std::vector<std::string> joint_names_msg = msg->joint_names;

    if (velocity.size() != joint_names_msg.size())
    {
        ROS_ERROR_STREAM(
            "Error in converter node: msg.velocity has hot the same "
            "size as msg.names! Message is not converted!"
            << std::endl);
        return;
    }

    if (_first_msg)
    {
        if (!_ur_utility->initJointMapping(joint_names_msg))
        {
            return;
        }
        _first_msg = false;
    }
    _ur_utility->parseJointStates(velocity, msg->points[0].velocities);

    std_msgs::Float64 send_msg;
    for (int i = 0; i < (int)velocity.size(); ++i)
    {
        send_msg.data = velocity[i];
        _pub_vel[i].publish(send_msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur_msgs_translation");
    ros::NodeHandle n("~");

    ROS_INFO("ur_msgs_translation started");

    _ur_utility = std::make_unique<mhp_robot::robot_misc::URUtility>();

    // create and fill vector of all publishers
    for (int i = 0; i < _ur_utility->getJointsCount(); ++i)
    {
        std::stringstream name_vel;
        name_vel << "joint" << i << "_velocity_controller/command";
        _pub_vel.push_back(n.advertise<std_msgs::Float64>(name_vel.str(), 1));
    }

    ros::Subscriber sub = n.subscribe("joint_speed", 1, &velocityCallback);

    ros::spin();
    return 0;
}
