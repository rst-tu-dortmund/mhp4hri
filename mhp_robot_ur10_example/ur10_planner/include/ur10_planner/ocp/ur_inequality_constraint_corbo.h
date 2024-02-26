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

#ifndef UR_INEQUALITY_CONSTRAINT_MHP_PLANNER_H
#define UR_INEQUALITY_CONSTRAINT_MHP_PLANNER_H

#include <mhp_planner/ocp/functions/stage_functions.h>
#include <ur10_planner/ocp/constraints/ur_base_acceleration_constraint_corbo.h>
#include <ur10_planner/ocp/constraints/ur_base_collision_constraint_corbo.h>
#include <ur_utilities/ur_collision/ur_collision.h>

namespace mhp_planner {

class URInequalityConstraint : public StageInequalityConstraint
{
 public:
    using Ptr  = std::shared_ptr<URInequalityConstraint>;
    using UPtr = std::unique_ptr<URInequalityConstraint>;

    URInequalityConstraint() = default;

    StageInequalityConstraint::Ptr getInstance() const override;
    int getNonIntegralStateTermDimension(int k) const override;
    int getNonIntegralControlDeviationTermDimension(int k) const override;

    void computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const override;

    void computeNonIntegralControlDeviationTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_prev,
                                                double dt, Eigen::Ref<Eigen::VectorXd> cost) const override;

    bool update(int n, double t, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref, ReferenceTrajectoryInterface* sref,
                bool single_dt, const Eigen::VectorXd& x0, StagePreprocessor::Ptr stage_preprocessor, const std::vector<double>& dts,
                const DiscretizationGridInterface*) override;

    void setPlannerId(const int id) override { _collision_constraint->setPlannerId(id); }
    int getPlannerId() const override { return _collision_constraint->getPlannerId(); }
    bool isPlannerSet() const override { return _collision_constraint->isPlannerSet(); }

    bool fromParameterServer(const std::string& ns) override;

 private:
    using RobotCollisionConstraint = mhp_robot::robot_trajectory_optimization::RobotCollisionConstraint;
    using URCollision              = mhp_robot::robot_collision::URCollision;

    URBaseAccelerationConstraint::Ptr _acceleration_constraint;
    URBaseCollisionConstraint::Ptr _collision_constraint;
    int _dimension = 0;
};

FACTORY_REGISTER_STAGE_INEQUALITIES(URInequalityConstraint)

}  // namespace mhp_planner
#endif  // UR_INEQUALITY_CONSTRAINT_MHP_PLANNER_H
