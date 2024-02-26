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

#ifndef BASE_FILTER_H
#define BASE_FILTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl_filters {

/**
 * @brief The Base filter class
 *
 * Basefilter class is the base class for pointcloud filters we want to apply for the * * * octomap.
 *
 * @author Heiko Renz
 */

class BaseFilter
{
 public:
    using Ptr  = std::shared_ptr<BaseFilter>;
    using UPtr = std::unique_ptr<BaseFilter>;

    BaseFilter() = default;

    BaseFilter(const BaseFilter&)            = delete;
    BaseFilter(BaseFilter&&)                 = default;
    BaseFilter& operator=(const BaseFilter&) = delete;
    BaseFilter& operator=(BaseFilter&&)      = default;
    virtual ~BaseFilter() {}

    virtual void filter(pcl::PCLPointCloud2* input_pc, pcl::PCLPointCloud2* output_pc) = 0;
    virtual bool initialize() = 0;

 protected:
    bool _initialized = false;
};
}  // namespace pcl_filters

#endif  // BASE_INTERPOLATION_H
