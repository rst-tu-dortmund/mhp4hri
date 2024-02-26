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
#ifndef PCL_FILTER_PROCESSOR_H
#define PCL_FILTER_PROCESSOR_H

#include <ros/ros.h>
#include <pcl_filters/base_filter.h>

namespace pcl_filters {
class PCLFilterProcessor
{
 public:
    using Ptr  = std::shared_ptr<PCLFilterProcessor>;
    using UPtr = std::unique_ptr<PCLFilterProcessor>;

    PCLFilterProcessor(std::vector<pcl_filters::BaseFilter::UPtr>& filters);

    PCLFilterProcessor(const PCLFilterProcessor&)            = delete;
    PCLFilterProcessor(PCLFilterProcessor&&)                 = default;
    PCLFilterProcessor& operator=(const PCLFilterProcessor&) = delete;
    PCLFilterProcessor& operator=(PCLFilterProcessor&&)      = default;
    ~PCLFilterProcessor()                                        = default;

    void processFilters(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    bool initializeFilters();

 private:
    std::vector<pcl_filters::BaseFilter::UPtr> _filters;

    ros::Publisher _filtered_cloud_pub;
};
}  // namespace pcl_filters
#endif  // PCL_FILTER_PROCESSOR_H