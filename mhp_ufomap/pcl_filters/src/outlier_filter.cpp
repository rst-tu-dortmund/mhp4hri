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

#include <pcl_filters/outlier_filter.h>

namespace pcl_filters {
OutlierFilter::OutlierFilter(double radius, int neigbours)
{
    _radius    = radius;
    _neigbours = neigbours;
}

void OutlierFilter::filter(pcl::PCLPointCloud2* input_pc, pcl::PCLPointCloud2* output_pc)
{
    if (!_initialized)
    {
        ROS_ERROR("OutlierFilter: Filter not initialized!");
    }
    _cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(*input_pc, *_cloud);
    // Check for invalid points (NaN or Inf)
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*_cloud, *_cloud, indices);

    // pcl::PCLPointCloud2ConstPtr input_ptr(input_pc);

    _outlier_filter.setInputCloud(_cloud);
    _outlier_filter.filter(*filteredCloud);

    pcl::toPCLPointCloud2(*filteredCloud, *output_pc);
}

bool OutlierFilter::initialize()
{
    _outlier_filter.setRadiusSearch(_radius);             // Adjust the search radius as needed
    _outlier_filter.setMinNeighborsInRadius(_neigbours);  // Adjust the minimum number of neighbors as needed

    _outlier_filter.setKeepOrganized(true);  // Set to true to keep the point cloud organized

    _initialized = true;

    return true;
}
}  // namespace pcl_filters
