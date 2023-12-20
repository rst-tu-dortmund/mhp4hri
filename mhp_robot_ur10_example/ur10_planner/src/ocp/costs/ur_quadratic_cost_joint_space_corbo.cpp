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

#include <ur10_planner/ocp/costs/ur_quadratic_cost_joint_space_corbo.h>

namespace mhp_planner {

mhp_planner::URBaseCostFunction::Ptr URQuadraticCostJointSpace::getInstance() const { return std::make_shared<URQuadraticCostJointSpace>(); }

bool URQuadraticCostJointSpace::fromParameterServer(const std::string& ns)
{
    ros::NodeHandle nh;

    // Set Weight matrices
    std::vector<double> Q_vec, R_vec, Rd_vec;
    nh.getParam(ns + "/Q", Q_vec);
    nh.getParam(ns + "/R", R_vec);
    nh.getParam(ns + "/Rd", Rd_vec);

    _Q_diag  = Eigen::Map<const Eigen::Matrix<double, -1, 1>>(Q_vec.data(), Q_vec.size()).asDiagonal();
    _R_diag  = Eigen::Map<const Eigen::Matrix<double, -1, 1>>(R_vec.data(), R_vec.size()).asDiagonal();
    _Rd_diag = Eigen::Map<const Eigen::Matrix<double, -1, 1>>(Rd_vec.data(), Rd_vec.size()).asDiagonal();

    // others
    nh.getParam(ns + "/evaluate_controls", _evaluate_controls);
    nh.getParam(ns + "/evaluate_control_deviation", _evaluate_control_deviation);
    nh.getParam(ns + "/evaluate_states", _evaluate_states);

    return true;
}

}  // namespace mhp_planner
