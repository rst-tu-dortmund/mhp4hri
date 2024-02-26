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

#ifndef OUTLIER_FILTER_H
#define OUTLIER_FILTER_H

#include <pcl_filters/base_filter.h>
#include <pcl/filters/radius_outlier_removal.h>


namespace pcl_filters {
class OutlierFilter : public BaseFilter
{
    public:
    using Ptr  = std::shared_ptr<OutlierFilter>;
    using UPtr = std::unique_ptr<OutlierFilter>;

    OutlierFilter(double radius=0.3, int neigbours=20);
    void filter(pcl::PCLPointCloud2* input_pc, pcl::PCLPointCloud2* output_pc) override;
    bool initialize() override;

    private:   
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> _outlier_filter;
    double _radius = 0.3;
    int _neigbours = 20;
};

} // namespace pcl_filters

#endif  // OUTLIER_FILTER_H
