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

#ifndef ROBOT_COLLISION_POTENTIAL_H
#define ROBOT_COLLISION_POTENTIAL_H

#include <mhp_robot/robot_collision/robot_collision.h>
#include <mhp_robot/robot_misc/planning_delay.h>
#include <mhp_robot/robot_obstacle/obstacle_list.h>
#include <mhp_robot/robot_trajectory_optimization/robot_stage_preprocessor.h>
#include <Eigen/Core>
#include <memory>

namespace mhp_robot {
namespace robot_trajectory_optimization {

class RobotCollisionPotential
{
 public:
    using Ptr  = std::shared_ptr<RobotCollisionPotential>;
    using UPtr = std::unique_ptr<RobotCollisionPotential>;

    RobotCollisionPotential() = default;

    RobotCollisionPotential(const RobotCollisionPotential&)            = delete;
    RobotCollisionPotential(RobotCollisionPotential&&)                 = delete;
    RobotCollisionPotential& operator=(const RobotCollisionPotential&) = delete;
    RobotCollisionPotential& operator=(RobotCollisionPotential&&)      = delete;
    virtual ~RobotCollisionPotential() {}

    virtual double computePotentials(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k) = 0;

    virtual void computeGradient(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> dx);

    virtual void computeHessian(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::MatrixXd> dxdx);

    bool initialize(RobotStagePreprocessor::Ptr preprocessor, robot_collision::RobotCollision::UPtr robot_collision);
    bool update(double dt);
    bool isInitialized() const;

    void setPlannerId(const int id)
    {
        _planner_id = id;
        if (_planner_id != 0) _ms_planner_mode = true;
    }
    int getPlannerId() const { return _planner_id; }
    bool isPlannerSet() const { return _ms_planner_mode; }

 protected:
    using ObstacleList   = robot_obstacle::ObstacleList;
    using RobotCollision = robot_collision::RobotCollision;
    using PlanningDelay  = robot_misc::PlanningDelay;

    void computeDistances(const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::ArrayXd> rc, Eigen::Ref<Eigen::ArrayXd> gc,
                          Eigen::Ref<Eigen::ArrayXd> sc, Eigen::Ref<Eigen::ArrayXd> soc, Eigen::Ref<Eigen::ArrayXd> doc,
                          Eigen::Ref<Eigen::ArrayXd> poc, Eigen::Ref<Eigen::ArrayXd> hhc, Eigen::Ref<Eigen::ArrayXd> htc,
                          Eigen::Ref<Eigen::ArrayXd> hlc, Eigen::Ref<Eigen::ArrayXd> huh, Eigen::Ref<Eigen::ArrayXd> hut,
                          Eigen::Ref<Eigen::ArrayXd> hul, double t);

    RobotCollision::UPtr _robot_collision;
    RobotStagePreprocessor::Ptr _preprocessor;
    ObstacleList _obstacle_manager;
    PlanningDelay _planning_delay;

    std::vector<robot_misc::Obstacle> _static_obstacle_list;
    std::vector<robot_misc::Obstacle> _dynamic_obstacle_list;
    std::vector<robot_misc::Plane> _plane_obstacle_list;
    std::vector<robot_misc::Human> _human_obstacle_list;

    Eigen::VectorXd _weights = Eigen::VectorXd::Ones(9);
    Eigen::VectorXd _margins = Eigen::VectorXd::Ones(9) * 0.2;

    int _ground_collision_dimension                  = 0;
    int _roof_collision_dimension                    = 0;
    int _self_collision_dimension                    = 0;
    int _static_obstacle_dimension                   = 0;
    int _dynamic_obstacle_dimension                  = 0;
    int _plane_collision_dimension                   = 0;
    int _human_head_collision_dimension              = 0;
    int _human_trunk_collision_dimension             = 0;
    int _human_limbs_collision_dimension             = 0;
    int _human_uncertainty_head_collision_dimension  = 0;
    int _human_uncertainty_trunk_collision_dimension = 0;
    int _human_uncertainty_limbs_collision_dimension = 0;
    double _dt                                       = 0.1;
    double _eps                                      = 1e-7;

    bool _initialized                  = false;
    bool _ground_collision             = true;
    bool _roof_collision               = true;
    bool _self_collision               = true;
    bool _static_obstacles             = true;
    bool _dynamic_obstacles            = true;
    bool _human_head_collision         = true;
    bool _human_trunk_collision        = true;
    bool _human_limbs_collision        = true;
    bool _plane_collision              = true;
    bool _min_distances                = false;
    bool _use_preprocessor             = false;
    bool _obstacle_prediction          = false;
    bool _prediction_offest            = false;
    bool _uncertainty_estimation_human = false;

    // Variables for Multistage Planner
    int _planner_id       = 0;
    bool _ms_planner_mode = false;

    enum UncertaintyMode { NoUncertaintyEstimation, SkeletonSplitting, RadiusIncrease } _uncertainty_mode = NoUncertaintyEstimation;
};

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot

#endif  // ROBOT_COLLISION_POTENTIAL_H
