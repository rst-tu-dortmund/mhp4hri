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

#ifndef ROBOT_COLLISION_CONSTRAINT_H
#define ROBOT_COLLISION_CONSTRAINT_H

#include <mhp_robot/robot_misc/planning_delay.h>
#include <mhp_robot/robot_trajectory_optimization/robot_stage_preprocessor.h>
#include <Eigen/Core>
#include <memory>
#include <tuple>

namespace mhp_robot {
namespace robot_trajectory_optimization {

class RobotCollisionConstraint
{
 public:
    using Ptr  = std::shared_ptr<RobotCollisionConstraint>;
    using UPtr = std::unique_ptr<RobotCollisionConstraint>;

    RobotCollisionConstraint() = default;

    RobotCollisionConstraint(const RobotCollisionConstraint&)            = delete;
    RobotCollisionConstraint(RobotCollisionConstraint&&)                 = delete;
    RobotCollisionConstraint& operator=(const RobotCollisionConstraint&) = delete;
    RobotCollisionConstraint& operator=(RobotCollisionConstraint&&)      = delete;
    virtual ~RobotCollisionConstraint() {}

    bool initialize(RobotStagePreprocessor::Ptr preprocessor, robot_collision::RobotCollision::UPtr robot_collision);
    void update(double dt);

    virtual void compute(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost);
    virtual int getDimension() const;

    bool isInitialized() const;

    void setPlannerId(const int id)
    {
        _planner_id = id;
        if (_planner_id != 0) _ms_planner_mode = true;
    }
    int getPlannerId() const { return _planner_id; }
    bool isPlannerSet() const { return _ms_planner_mode; }

 protected:
    using RobotCollision = robot_collision::RobotCollision;
    using ObstacleList   = robot_obstacle::ObstacleList;
    using PlanningDelay  = robot_misc::PlanningDelay;

    RobotCollision::UPtr _robot_collision;
    RobotStagePreprocessor::Ptr _preprocessor;
    ObstacleList _obstacle_manager;
    PlanningDelay _planning_delay;

    std::vector<robot_misc::Obstacle> _static_obstacle_list;
    std::vector<robot_misc::Obstacle> _dynamic_obstacle_list;
    std::vector<robot_misc::Plane> _plane_collision_list;
    std::vector<robot_misc::Human> _human_obstacle_list;

    bool _ground_collision      = false;
    bool _roof_collision        = false;
    bool _self_collision        = true;
    bool _static_obstacles      = false;
    bool _dynamic_obstacles     = false;
    bool _human_head_collision  = false;
    bool _human_trunk_collision = false;
    bool _human_limbs_collision = false;
    bool _plane_collision       = false;
    bool _use_preprocessor      = false;
    bool _min_distances         = true;
    bool _obstacle_prediction   = false;
    bool _prediction_offset     = false;
    bool _initialized           = false;
    int _dimension              = 0;
    double _dt                  = 0.1;

    double _ground_collision_threshold = 0.05;
    double _roof_collision_threshold   = 0.05;
    double _self_collision_threshold   = 0.02;
    double _static_obstacle_threshold  = 0.05;
    double _dynamic_obstacle_threshold = 0.05;
    double _plane_collision_threshold  = 0.05;
    std::tuple<double, double, double> _human_obstacle_thresholds{0.2, 0.1, 0.05};  // head, trunk, limbs

    // Variables for Multistage Planner
    int _planner_id       = 0;
    bool _ms_planner_mode = false;
};

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot

#endif  // ROBOT_COLLISION_CONSTRAINT_H
