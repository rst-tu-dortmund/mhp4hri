/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2024,
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
 *  Authors: Maximilian Krämer
 *  Modifier(s): Heiko Renz
 *********************************************************************/

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mhp_robot/SrvStartSimulation.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

bool _start_replay = false;
bool _first_run    = true;

bool startReplayCallback(mhp_robot::SrvStartSimulation::Request& request, mhp_robot::SrvStartSimulation::Response& response)
{
    _start_replay   = request.start;
    _first_run      = true;
    response.result = true;
    return true;
}

rosbag::View::iterator getStart(rosbag::View& view, double start_time)
{
    rosbag::View::iterator it = view.begin();
    bool first                = true;
    ros::Time bag_start_time;

    while (it != view.end())
    {
        const rosbag::MessageInstance m        = *it;
        sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
        const sensor_msgs::PointCloud2& pcl    = *msg;

        if (first)
        {
            bag_start_time = pcl.header.stamp;
            first          = false;
        }

        if ((pcl.header.stamp - bag_start_time).toSec() >= start_time)
        {
            return it;
        }
        it++;
    }
    return view.end();
}

int main(int argc, char** argv)
{
#ifndef NDEBUG
    sleep(5);
#endif
    ros::init(argc, argv, "robot_pcl_bag_parser");
    ros::NodeHandle n("~");

    // retrieve params
    std::string rosbag_name = "";
    std::string rosbag_path = "";
    double start_time       = 0;
    bool loop               = true;

    n.getParam("/rosbag_pcl/rosbag_name", rosbag_name);

    n.getParam("/rosbag_pcl/bag_start_time", start_time);
    n.getParam("/rosbag_pcl/bag_loop", loop);

    if (rosbag_name.compare("") == 0)
    {
        // no bag file set
        ROS_WARN("RobotTFBagParser: No file set. Exiting...");
        return 0;
    }

    // load bag
    rosbag::Bag bag;
    bag.open(rosbag_name);  // read is default
    rosbag::View bag_viewer(bag, rosbag::TopicQuery("points2"));
    int N                        = bag_viewer.size();
    rosbag::View::iterator start = getStart(bag_viewer, start_time);

    if (N < 1)
    {
        // "empty" bag
        ROS_WARN("RobotPCLBagParser: Empty bag. Exiting...");
        return 0;
    }

    // prepare
    ros::Publisher pcl_pub;
    pcl_pub = n.advertise<sensor_msgs::PointCloud2>("points_stamped", 10);
    sensor_msgs::PointCloud2 pcl_msg;

    // prepare start_service
    ros::ServiceServer service_server = n.advertiseService("start_pcl_replay", &startReplayCallback);

    // main loop
    ros::Time sim_start_time, bag_start_time, current_time;

    while (ros::ok())
    {
        ros::spinOnce();

        if (_start_replay && start != bag_viewer.end())
        {
            current_time = ros::Time::now();

            // extract msg
            const rosbag::MessageInstance m        = *start;
            sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
            pcl_msg                                = *msg;
            if (_first_run)
            {
                // save time stamp of replay start
                sim_start_time = current_time;
                bag_start_time = pcl_msg.header.stamp;
                _first_run     = false;
            }

            // calculate timing
            double replay_dt = (current_time - sim_start_time).toSec();
            double bag_dt    = (pcl_msg.header.stamp - bag_start_time).toSec();

            if ((replay_dt >= bag_dt))
            {
                // time to send next msg
                pcl_msg.header.stamp = ros::Time::now();
                pcl_msg.header.frame_id = "fixed_depth_camera";
                pcl_pub.publish(pcl_msg);

                start++;
            }
        }

        if (loop && (start == bag_viewer.end()))
        {
            // restart
            start      = getStart(bag_viewer, start_time);
            _first_run = true;
        }
    }

    return 0;
}
