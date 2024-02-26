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

#ifndef UR_STAGE_COST_MHP_PLANNER_H_
#define UR_STAGE_COST_MHP_PLANNER_H_

#include <mhp_planner/ocp/functions/stage_functions.h>
#include <ur10_planner/ocp/costs/ur_base_cost_function_corbo.h>
#include <ur10_planner/ocp/potentials/ur_base_collision_potential_corbo.h>
#include <ur10_planner/ocp/gains/ur_base_information_gain_corbo.h>
#include <ur_utilities/ur_collision/ur_collision.h>
#include <ur_utilities/ur_kinematic/ur_kinematic.h>

namespace mhp_planner {

class URStageCost : public StageCost
{
 public:
    using Ptr  = std::shared_ptr<URStageCost>;
    using UPtr = std::unique_ptr<URStageCost>;

    URStageCost() = default;

    StageCost::Ptr getInstance() const override;
    int getNonIntegralStateTermDimension(int k) const override;
    int getNonIntegralControlTermDimension(int k) const override;
    int getNonIntegralStateControlTermDimension(int k) const override;
    int getNonIntegralControlDeviationTermDimension(int k) const override;

    void computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const override;

    void computeNonIntegralControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& u_k, Eigen::Ref<Eigen::VectorXd> cost) const override;

    void computeNonIntegralStateControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                            Eigen::Ref<Eigen::VectorXd> cost) const override;

    void computeNonIntegralControlDeviationTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_prev,
                                                double dt, Eigen::Ref<Eigen::VectorXd> cost) const override;

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
    URBaseInformationGain::Ptr _information_gain;
    
    const ReferenceTrajectoryInterface* _x_ref = nullptr;
    const ReferenceTrajectoryInterface* _u_ref = nullptr;
    const ReferenceTrajectoryInterface* _s_ref = nullptr;

    int _planner_id = 0;
};

FACTORY_REGISTER_STAGE_COST(URStageCost)

}  // namespace mhp_planner

#endif  // UR_STAGE_COST_MHP_PLANNER_H_
