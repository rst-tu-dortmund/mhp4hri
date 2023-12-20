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
#include <ur10_planner/ocp/ur_final_state_constraint_joint_space_corbo.h>

namespace mhp_planner {

FinalStageConstraint::Ptr URFinalStateConstraintJointSpace::getInstance() const { return std::make_shared<URFinalStateConstraintJointSpace>(); }

bool URFinalStateConstraintJointSpace::fromParameterServer(const std::string& ns)
{
    ros::NodeHandle nh;

    // weight
    std::vector<double> S;
    nh.getParam(ns + "/S", S);
    if (!setWeightS(Eigen::Map<const Eigen::Matrix<double, -1, 1>>(S.data(), S.size()).asDiagonal()))
    {
        PRINT_ERROR("URFinalStateConstraintJointSpace: cannot set diagonal weight matrix S.");
        return false;
    }

    // gamma
    nh.getParam(ns + "/gamma", _gamma);

    return true;
}

}  // namespace mhp_planner
