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
#include <ur10_planner/ocp/constraints/ur_collision_constraint_corbo.h>

namespace mhp_planner {

URBaseCollisionConstraint::Ptr URCollisionConstraint::getInstance() const { return std::make_shared<URCollisionConstraint>(); }

bool URCollisionConstraint::fromParameterServer(const std::string& ns)
{
    ros::NodeHandle nh;

    // Get the collision avoidance parameters
    nh.getParam(ns + "/ground_collision", _ground_collision);
    nh.getParam(ns + "/roof_collision", _roof_collision);
    nh.getParam(ns + "/self_collision", _self_collision);
    nh.getParam(ns + "/static_obstacles", _static_obstacles);
    nh.getParam(ns + "/dynamic_obstacles", _dynamic_obstacles);
    nh.getParam(ns + "/plane_collision", _plane_collision);
    nh.getParam(ns + "/human_head_collision", _human_head_collision);
    nh.getParam(ns + "/human_trunk_collision", _human_trunk_collision);
    nh.getParam(ns + "/human_limbs_collision", _human_limbs_collision);

    // Get the collision avoidance thresholds
    nh.getParam(ns + "/ground_collision_threshold", _ground_collision_threshold);
    nh.getParam(ns + "/roof_collision_threshold", _roof_collision_threshold);
    nh.getParam(ns + "/self_collision_threshold", _self_collision_threshold);
    nh.getParam(ns + "/static_obstacle_threshold", _static_obstacle_threshold);
    nh.getParam(ns + "/dynamic_obstacle_threshold", _dynamic_obstacle_threshold);
    nh.getParam(ns + "/plane_collision_threshold", _plane_collision_threshold);
    nh.getParam(ns + "/human_head_collision_threshold", std::get<0>(_human_obstacle_thresholds));
    nh.getParam(ns + "/human_trunk_collision_threshold", std::get<1>(_human_obstacle_thresholds));
    nh.getParam(ns + "/human_limbs_collision_threshold", std::get<2>(_human_obstacle_thresholds));

    // Get additonal parameters
    nh.getParam(ns + "/use_preprocessor", _use_preprocessor);
    nh.getParam(ns + "/min_distances", _min_distances);
    nh.getParam(ns + "/obstacle_prediction", _obstacle_prediction);
    nh.getParam(ns + "/prediction_offset", _prediction_offset);

    return true;
}
}  // namespace mhp_planner
