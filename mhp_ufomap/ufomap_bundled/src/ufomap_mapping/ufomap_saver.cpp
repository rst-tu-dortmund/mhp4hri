
/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2023,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *  Authors: Heiko Renz
 *********************************************************************/

#include <ros/ros.h>
#include <std_msgs/Bool.h>
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ufomap_saver");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    std::string mapFilename("");
    if (argc == 3 && strcmp(argv[1], "-f") == 0)
    {
        mapFilename = std::string(argv[2]);
    }
    else
    {
        ROS_ERROR("ufomap_saver: Usage: ufomap_saver [-f] mapFilename.ufomap");
        exit(1);
    }
    nh.setParam("map_filename", mapFilename);

    ros::Publisher save_map_topic_pub;
    save_map_topic_pub = nh.advertise<std_msgs::Bool>("/save_map_topic", 10);

    std_msgs::Bool save_map_topic_msg;
    save_map_topic_msg.data = true;

    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok() && count < 10)
    {
        count++;
        save_map_topic_pub.publish(save_map_topic_msg);
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}