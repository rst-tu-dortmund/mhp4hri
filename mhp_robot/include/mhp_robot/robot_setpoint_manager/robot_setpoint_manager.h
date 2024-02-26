/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  This software is currently not released.
 *  Redistribution and use in source and binary forms,
 *  with or without modification, are prohibited.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Maximilian Krämer
 *********************************************************************/

#ifndef ROBOT_SETPOINT_MANAGER_H
#define ROBOT_SETPOINT_MANAGER_H

#include <mhp_robot/robot_collision/robot_collision.h>
#include <mhp_robot/robot_misc/obstacle.h>
#include <mhp_robot/robot_obstacle/obstacle_list.h>
#include <mhp_robot/robot_setpoint_manager/objectives/base_setpoint_objective.h>
#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace mhp_robot {
namespace robot_set_point_manager {

class RobotSetPointManager
{
 public:
    using Ptr  = std::shared_ptr<RobotSetPointManager>;
    using UPtr = std::unique_ptr<RobotSetPointManager>;

    RobotSetPointManager(robot_collision::RobotCollision::UPtr robot_collision);

    RobotSetPointManager(const RobotSetPointManager&) = delete;
    RobotSetPointManager(RobotSetPointManager&&)      = delete;
    RobotSetPointManager& operator=(const RobotSetPointManager&) = delete;
    RobotSetPointManager& operator=(RobotSetPointManager&&) = delete;
    virtual ~RobotSetPointManager() {}

    void setJointSpaceWaypoints(const std::vector<Eigen::MatrixXd>& waypoints, const std::vector<double>& time);
    void validate();
    void setVelocityLimits(double qd_min, double qd_max);
    void setCollisionThresholds(double self_collision, double obstacle_collision, double ground_collision, double roof_collision,
                                double plane_collision, double human_collision);
    void considerDynamics(bool obstacle = true, bool human = true);
    bool getOptimalWaypoints(const objectives::BaseSetpointObjective& objective, std::vector<Eigen::VectorXd>& waypoints);
    bool getOptimalWaypoints(std::vector<Eigen::VectorXd>& waypoints);

 protected:
    using RobotUtility   = robot_misc::RobotUtility;
    using RobotCollision = robot_collision::RobotCollision;
    using Obstacle       = robot_misc::Obstacle;
    using ObstacleList   = robot_obstacle::ObstacleList;

    RobotCollision::UPtr _robot_collision;
    ObstacleList _obstacle_manager;

    std::vector<Eigen::MatrixXd> _waypoints;
    std::vector<double> _time;
    std::vector<std::vector<bool>> _selector;
    std::vector<std::vector<std::pair<int, double>>> _dp_table;

    double _qd_min                 = -0.5;
    double _qd_max                 = 0.5;
    double _min_self_collision     = 0.01;
    double _min_obstacle_collision = 0.01;
    double _min_ground_collision   = 0.01;
    double _min_roof_collision     = 0.01;
    double _min_plane_collision    = 0.01;
    double _min_human_collision    = 0.01;
    bool _dynamic_obstacles        = false;
    bool _human_obstacles          = false;

    std::vector<double> _q_min;
    std::vector<double> _q_max;

    virtual void validateJointState(const Eigen::Ref<const Eigen::MatrixXd>& solutions, std::vector<bool>& selector);
    virtual bool checkContinuity(const Eigen::Ref<const Eigen::VectorXd>& current, const Eigen::Ref<const Eigen::VectorXd>& next, double time) const;
};

}  // namespace robot_set_point_manager
}  // namespace mhp_robot

#endif  // ROBOT_SET_POINT_MANAGER_H
