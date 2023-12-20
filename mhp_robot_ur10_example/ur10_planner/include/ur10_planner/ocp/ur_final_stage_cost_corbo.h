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

#ifndef UR_FINAL_STAGE_COST_MHP_PLANNER_H
#define UR_FINAL_STAGE_COST_MHP_PLANNER_H

#include <mhp_planner/ocp/functions/stage_functions.h>
#include <ur10_planner/ocp/costs/ur_base_cost_function_corbo.h>
#include <ur10_planner/ocp/potentials/ur_base_collision_potential_corbo.h>
#include <ur10_planner/ocp/ur_stage_preprocessor_corbo.h>
#include <ur_utilities/ur_collision/ur_collision.h>
#include <ur_utilities/ur_kinematic/ur_kinematic.h>

namespace mhp_planner {

class URFinalStageCost : public FinalStageCost
{
 public:
    using Ptr  = std::shared_ptr<URFinalStageCost>;
    using UPtr = std::unique_ptr<URFinalStageCost>;

    URFinalStageCost() = default;

    FinalStageCost::Ptr getInstance() const override;

    int getNonIntegralStateTermDimension(int k) const override;

    void computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const override;

    bool update(int n, double t, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref, ReferenceTrajectoryInterface* sref,
                bool single_dt, const Eigen::VectorXd& x0, StagePreprocessor::Ptr stage_preprocessor, const std::vector<double>& dts,
                const DiscretizationGridInterface*) override;

    bool checkParameters(int state_dim, int control_dim) const override;

    void setPlannerId(const int id) override;
    int getPlannerId() const override;
    bool isPlannerSet() const override;

    bool fromParameterServer(const std::string& ns) override;

 private:
    using URCollision = mhp_robot::robot_collision::URCollision;
    using URKinematic = mhp_robot::robot_kinematic::URKinematic;

    URBaseCostFunction::Ptr _cost_function;
    URBaseCollisionPotential::Ptr _collision_potential;

    const ReferenceTrajectoryInterface* _x_ref = nullptr;
    const ReferenceTrajectoryInterface* _u_ref = nullptr;
    const ReferenceTrajectoryInterface* _s_ref = nullptr;
};  // namespace mhp_planner
FACTORY_REGISTER_FINAL_STAGE_COST(URFinalStageCost)

}  // namespace mhp_planner

#endif  // UR_FINAL_STAGE_COST_MHP_PLANNER_H
