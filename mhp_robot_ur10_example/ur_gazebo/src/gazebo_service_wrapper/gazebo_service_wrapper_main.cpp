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

#include <gazebo_msgs/ApplyBodyWrench.h>
#include <ros/ros.h>
#include <ur_msgs/ApplyBodyWrench.h>
#include <memory>

std::unique_ptr<ros::ServiceClient> srv_client;

void applyBodyWrenchCallback(const ur_msgs::ApplyBodyWrenchConstPtr msg)
{
    gazebo_msgs::ApplyBodyWrench srv;

    // copy into request
    srv.request.wrench          = msg->wrench;
    srv.request.duration        = msg->duration;
    srv.request.body_name       = msg->body_name;
    srv.request.start_time      = msg->start_time;
    srv.request.reference_frame = msg->reference_frame;
    srv.request.reference_point = msg->reference_point;

    if (!srv_client->call(srv))
    {
        ROS_WARN_STREAM("GazeboServiceWrapper: Failed to call service " << srv_client->getService());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gazebo_service_wrapper");
    ros::NodeHandle n("~");

    ROS_INFO("gazebo_service_wrapper started");

    // create service
    srv_client = std::make_unique<ros::ServiceClient>(n.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench"));

    // create subscriber
    ros::Subscriber sub = n.subscribe("apply_body_wrench", 1, &applyBodyWrenchCallback);

    ros::spin();
    return 0;
}
