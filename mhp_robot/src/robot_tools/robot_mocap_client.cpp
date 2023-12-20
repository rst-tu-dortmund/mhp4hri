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

#include <mhp_robot/robot_mocap/robot_mocap.h>
#include <ros/ros.h>

namespace rosparam {
namespace keys {

const std::string MulticastIpAddress = "/robot_mocap/multicast_address";
const std::string MocapFrame         = "/robot_mocap/mocap_frame";
const std::string ServerIpAddress    = "/robot_mocap/server_address";
const std::string DataPort           = "/robot_mocap/data_port";
const std::string CommandPort        = "/robot_mocap/command_port";
const std::string Relative           = "/robot_mocap/relative";
const std::string PredictionDt       = "/robot_mocap/pred_dt";
const std::string PredictionN        = "/robot_mocap/pred_N";

}  // namespace keys
}  // namespace rosparam

int main(int argc, char** argv)
{
    // sleep(5);

    ros::init(argc, argv, "robot_mocap");
    ros::NodeHandle nh("~");

    ROS_INFO("MoCap client running...");

    // Default params
    std::string server_address    = "10.10.0.101";
    std::string multicast_address = "239.0.0.1";
    int data_port                 = 1511;
    int command_port              = 1510;
    std::string mocap_frame       = "/mocap";
    bool relative                 = false;
    int pred_N                    = 0;
    double pred_dt                = 0.1;

    // Parse parms
    if (nh.hasParam(rosparam::keys::MocapFrame))
    {
        nh.getParam(rosparam::keys::MocapFrame, mocap_frame);
    }
    else
    {
        ROS_WARN_STREAM("Could not get mocap_frame, using default: " << mocap_frame);
    }

    if (nh.hasParam(rosparam::keys::MulticastIpAddress))
    {
        nh.getParam(rosparam::keys::MulticastIpAddress, multicast_address);
    }
    else
    {
        ROS_WARN_STREAM("Could not get multicast address, using default: " << multicast_address);
    }

    if (nh.hasParam(rosparam::keys::ServerIpAddress))
    {
        nh.getParam(rosparam::keys::ServerIpAddress, server_address);
    }
    else
    {
        ROS_WARN_STREAM("Could not get server address, using default: " << server_address);
    }

    if (nh.hasParam(rosparam::keys::CommandPort))
    {
        nh.getParam(rosparam::keys::CommandPort, command_port);
    }
    else
    {
        ROS_WARN_STREAM("Could not get command port, using default: " << command_port);
    }

    if (nh.hasParam(rosparam::keys::DataPort))
    {
        nh.getParam(rosparam::keys::DataPort, data_port);
    }
    else
    {
        ROS_WARN_STREAM("Could not get data port, using default: " << data_port);
    }

    if (nh.hasParam(rosparam::keys::Relative))
    {
        nh.getParam(rosparam::keys::Relative, relative);
    }
    else
    {
        ROS_WARN_STREAM("Could not get data relative flag, using default: " << relative);
    }

    if (nh.hasParam(rosparam::keys::PredictionN))
    {
        nh.getParam(rosparam::keys::PredictionN, pred_N);
    }
    else
    {
        ROS_WARN_STREAM("Could not get prediction steps, using default: " << pred_N);
    }

    if (nh.hasParam(rosparam::keys::PredictionDt))
    {
        nh.getParam(rosparam::keys::PredictionDt, pred_dt);
    }
    else
    {
        ROS_WARN_STREAM("Could not get prediction dt, using default: " << pred_dt);
    }

    // Create client object
    mhp_robot::robot_mocap::RobotMocap mocap(mocap_frame, relative, pred_N, pred_dt);

    // Connect
    if (!mocap.connect(server_address, multicast_address, command_port, data_port)) return 1;

    // Start publishing (blocking)
    mocap.publish();

    return 0;
}
