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

#include <pcl_filters/pcl_filter_processor.h>

namespace pcl_filters {

PCLFilterProcessor::PCLFilterProcessor(std::vector<pcl_filters::BaseFilter::UPtr>& filters)
{
    ros::NodeHandle _nh;
    _filters            = std::move(filters);
    _filtered_cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
}

void PCLFilterProcessor::processFilters(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2* input_pc  = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2* output_pc = new pcl::PCLPointCloud2;

    pcl_conversions::toPCL(*cloud_msg, *input_pc);

    for (auto& filter : _filters)
    {
        filter->filter(input_pc, output_pc);
        input_pc = output_pc;
    }
    if (_filters.empty())
    {
        ROS_WARN_ONCE("PCLFilterProcessor: No filters!");
        output_pc = input_pc;
    }
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(*output_pc, output);
    // Publish the data
    _filtered_cloud_pub.publish(output);
}
bool PCLFilterProcessor::initializeFilters()
{
    for (auto& filter : _filters)
    {
        filter->initialize();
    }
    return true;
}
}  // namespace pcl_filters