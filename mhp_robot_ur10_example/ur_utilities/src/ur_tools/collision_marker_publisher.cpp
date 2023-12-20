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

#include <mhp_robot/robot_collision/robot_collision_marker_publisher.h>
#include <ros/ros.h>
#include <ur_utilities/ur_collision/ur_collision.h>
#include <ur_utilities/ur_misc/ur_utility.h>

using RobotCollisionMarkerPublisher = mhp_robot::robot_collision::RobotCollisionMarkerPublisher;
using URCollision                   = mhp_robot::robot_collision::URCollision;
using URUtility                     = mhp_robot::robot_misc::URUtility;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur_collision_marker_publisher");

    ROS_INFO_STREAM("UR Collision Marker Publisher Started...");

    RobotCollisionMarkerPublisher publisher("/joint_states", "/robot_workspace_monitor/obstacles", std::make_unique<URCollision>(),
                                            std::make_unique<URUtility>());

    publisher.publish();

    ros::waitForShutdown();
    return 0;
}
