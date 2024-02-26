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

#include <ur10_planner/ocp/ur_stage_preprocessor_corbo.h>

namespace mhp_planner {

StagePreprocessor::Ptr URStagePreprocessor::getInstance() const { return std::make_shared<URStagePreprocessor>(); }

void URStagePreprocessor::precompute(const Eigen::Ref<const Eigen::VectorXd>& input)
{
    if (_ur_stage_preprocessor) _ur_stage_preprocessor->precompute(input);
}

bool URStagePreprocessor::update(int n, double t, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref,
                                 ReferenceTrajectoryInterface* sref, bool single_dt, const Eigen::VectorXd& x0, const std::vector<double>& dts,
                                 const DiscretizationGridInterface*)
{
    if (!_ur_stage_preprocessor)
    {
        _ur_stage_preprocessor = std::make_shared<RobotStagePreprocessor>(_static_obstacles, _self_collision, _roof_collision, _ground_collision,
                                                                          _plane_collision, std::make_unique<URCollision>());
    }
    _ur_stage_preprocessor->update();

    return false;
}

mhp_robot::robot_trajectory_optimization::RobotStagePreprocessor::Ptr URStagePreprocessor::getPreprocessor() { return _ur_stage_preprocessor; }

bool URStagePreprocessor::fromParameterServer(const std::string& ns)
{
    ros::NodeHandle nh;

    nh.getParam(ns + "/roof_collision", _roof_collision);
    nh.getParam(ns + "/ground_collision", _ground_collision);
    nh.getParam(ns + "/self_collision", _self_collision);
    nh.getParam(ns + "/static_obstacles", _static_obstacles);
    nh.getParam(ns + "/plane_collision", _plane_collision);

    if (_ur_stage_preprocessor) _ur_stage_preprocessor->set(_static_obstacles, _self_collision, _roof_collision, _ground_collision, _plane_collision);

    return true;
}

}  // namespace mhp_planner
