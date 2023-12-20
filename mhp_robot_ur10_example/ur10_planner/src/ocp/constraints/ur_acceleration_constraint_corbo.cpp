/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2023,
 *  TU Dortmund University, Institute of Control Theory and System Engineering
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
*  Maintainer(s)/Modifier(s): Heiko Renz
 *********************************************************************/
#include <ur10_planner/ocp/constraints/ur_acceleration_constraint_corbo.h>

namespace mhp_planner {

URBaseAccelerationConstraint::Ptr URAccelerationConstraint::getInstance() const { return std::make_shared<URAccelerationConstraint>(); }

bool URAccelerationConstraint::fromParameterServer(const std::string& ns)
{
    ros::NodeHandle nh;

    // Set active parameter
    nh.getParam(ns + "/active", _active);

    // Set limits
    std::vector<double> ud_max, ud_min;
    nh.getParam(ns + "/ud_ub", ud_max);
    nh.getParam(ns + "/ud_lb", ud_min);
    _max = Eigen::Map<const Eigen::ArrayXd>(ud_max.data(), ud_max.size());
    _min = Eigen::Map<const Eigen::ArrayXd>(ud_min.data(), ud_min.size());

    return true;
}

}  // namespace mhp_planner
