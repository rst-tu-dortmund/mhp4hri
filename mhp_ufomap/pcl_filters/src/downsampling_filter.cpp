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

#include <pcl_filters/downsampling_filter.h>

namespace pcl_filters {

DownsamplingFilter::DownsamplingFilter(double leaf_size) { _leaf_size = leaf_size; }
void DownsamplingFilter::filter(pcl::PCLPointCloud2* input_pc, pcl::PCLPointCloud2* output_pc)
{
    if (!_initialized)
    {
        ROS_ERROR("DownsamplingFilter: Filter not initialized!");
    }

    pcl::PCLPointCloud2ConstPtr input_ptr(input_pc);
    _voxel_grid_filter.setInputCloud(input_ptr);
    _voxel_grid_filter.filter(*output_pc);
}

bool DownsamplingFilter::initialize()
{
    _voxel_grid_filter.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
    _initialized = true;
    return true;
}
}  // namespace pcl_filters

