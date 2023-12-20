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
#ifndef UR_QUADRATIC_COST_JOINT_SPACE_MHP_PLANNER_H
#define UR_QUADRATIC_COST_JOINT_SPACE_MHP_PLANNER_H

#include <mhp_planner/core/factory.h>
#include <mhp_robot/robot_trajectory_optimization/robot_quadratic_cost_joint_space.h>
#include <ur10_planner/ocp/costs/ur_base_cost_function_corbo.h>

namespace mhp_planner {

class URQuadraticCostJointSpace : public URBaseCostFunction, public mhp_robot::robot_trajectory_optimization::RobotQuadraticCostJointSpace
{
 public:
    using Ptr  = std::shared_ptr<URQuadraticCostJointSpace>;
    using UPtr = std::unique_ptr<URQuadraticCostJointSpace>;

    URQuadraticCostJointSpace() = default;

    URBaseCostFunction::Ptr getInstance() const override;

    bool fromParameterServer(const std::string& ns) override;
};
FACTORY_REGISTER_OBJECT(URQuadraticCostJointSpace, URBaseCostFunction)

}  // namespace mhp_planner

#endif  // UR_QUADRATIC_COST_JOINT_SPACE_MHP_PLANNER_H
