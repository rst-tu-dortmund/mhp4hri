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

#include <pcl_filters/base_filter.h>
#include <pcl_filters/downsampling_filter.h>
#include <pcl_filters/pcl_filter_processor.h>
#include <pcl_filters/outlier_filter.h>
#include <ros/ros.h>

// void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "PCL_filter");
    ros::NodeHandle nh;

    bool downsampling;
    nh.param("/filter_node/downsampling", downsampling, true);
    bool remove_outliers;
    nh.param("/filter_node/remove_outliers", remove_outliers, true);

    double voxel_leaf_size;
    nh.param("/filter_node/voxel_leaf_size", voxel_leaf_size, 0.05);
    double outlier_radius;
    nh.param("/filter_node/outlier_radius", outlier_radius, 0.3);
    int outlier_min_neighbors;
    nh.param("/filter_node/outlier_min_neighbors", outlier_min_neighbors, 20);

    std::vector<pcl_filters::BaseFilter::UPtr> filters;

    if (downsampling)
    {
        pcl_filters::DownsamplingFilter::UPtr downsample_filter_ptr = std::make_unique<pcl_filters::DownsamplingFilter>(voxel_leaf_size);
        filters.push_back(std::move(downsample_filter_ptr));
    }

    if (remove_outliers)
    {
        pcl_filters::OutlierFilter::UPtr outlier_filter_ptr =
            std::make_unique<pcl_filters::OutlierFilter>(outlier_radius, outlier_min_neighbors);
        filters.push_back(std::move(outlier_filter_ptr));
    }
    pcl_filters::PCLFilterProcessor processor(filters);

    if (!processor.initializeFilters()) ROS_ERROR("Could not initialize filters!");

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input", 1, &pcl_filters::PCLFilterProcessor::processFilters, &processor);

    ros::spin();
}
