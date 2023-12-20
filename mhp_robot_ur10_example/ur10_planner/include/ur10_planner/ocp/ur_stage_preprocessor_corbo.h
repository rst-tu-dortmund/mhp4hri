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

#ifndef UR_STAGE_PREPROCESSOR_MHP_PLANNER_H
#define UR_STAGE_PREPROCESSOR_MHP_PLANNER_H

#include <mhp_planner/ocp/functions/stage_preprocessor.h>
#include <mhp_robot/robot_trajectory_optimization/robot_stage_preprocessor.h>
#include <ur_utilities/ur_collision/ur_collision.h>

namespace mhp_planner {

class URStagePreprocessor : public StagePreprocessor
{
 public:
    using Ptr  = std::shared_ptr<URStagePreprocessor>;
    using UPtr = std::unique_ptr<URStagePreprocessor>;

    URStagePreprocessor() = default;

    StagePreprocessor::Ptr getInstance() const override;

    void precompute(const Eigen::Ref<const Eigen::VectorXd>& input) override;

    bool update(int n, double t, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref, ReferenceTrajectoryInterface* sref,
                bool single_dt, const Eigen::VectorXd& x0, const std::vector<double>& dts, const DiscretizationGridInterface*) override;

    mhp_robot::robot_trajectory_optimization::RobotStagePreprocessor::Ptr getPreprocessor();

    bool fromParameterServer(const std::string& ns) override;

 private:
    using RobotStagePreprocessor = mhp_robot::robot_trajectory_optimization::RobotStagePreprocessor;
    using URCollision            = mhp_robot::robot_collision::URCollision;

    RobotStagePreprocessor::Ptr _ur_stage_preprocessor;

    bool _ground_collision = true;
    bool _roof_collision   = true;
    bool _self_collision   = true;
    bool _static_obstacles = true;
    bool _plane_collision  = true;
};

FACTORY_REGISTER_STAGE_PREPROCESSOR(URStagePreprocessor)

}  // namespace mhp_planner

#endif  // UR_STAGE_PREPROCESSOR_MHP_PLANNER_H
