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

#ifndef DOWNSAMPLING_FILTER_H
#define DOWNSAMPLING_FILTER_H

#include <pcl_filters/base_filter.h>
#include <pcl/filters/voxel_grid.h>

namespace pcl_filters {
class DownsamplingFilter : public BaseFilter
{
 public:
    using Ptr  = std::shared_ptr<DownsamplingFilter>;
    using UPtr = std::unique_ptr<DownsamplingFilter>;

    DownsamplingFilter(double leaf_size=0.01);
    void filter(pcl::PCLPointCloud2* input_pc, pcl::PCLPointCloud2* output_pc) override;
    bool initialize() override;

 private:
    pcl::VoxelGrid<pcl::PCLPointCloud2> _voxel_grid_filter;
    double _leaf_size;
};

}  // namespace pcl_filters

#endif  // DOWNSAMPLING_FILTER_H