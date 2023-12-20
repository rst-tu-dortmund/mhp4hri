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

#ifndef ROBOT_STAGE_PREPROCESSOR_H
#define ROBOT_STAGE_PREPROCESSOR_H

#include <mhp_robot/robot_collision/robot_collision.h>
#include <mhp_robot/robot_kinematic/robot_kinematic.h>
#include <mhp_robot/robot_misc/robot_utility.h>
#include <mhp_robot/robot_obstacle/obstacle_list.h>
#include <memory>

namespace mhp_robot {
namespace robot_trajectory_optimization {

class RobotStagePreprocessor
{
 public:
    using Ptr  = std::shared_ptr<RobotStagePreprocessor>;
    using UPtr = std::unique_ptr<RobotStagePreprocessor>;

    RobotStagePreprocessor(bool static_obstacle, bool self_collision, bool roof_collision, bool ground_collision, bool plane_collision,
                           robot_collision::RobotCollision::UPtr robot_collision);

    RobotStagePreprocessor(const RobotStagePreprocessor&)            = delete;
    RobotStagePreprocessor(RobotStagePreprocessor&&)                 = delete;
    RobotStagePreprocessor& operator=(const RobotStagePreprocessor&) = delete;
    RobotStagePreprocessor& operator=(RobotStagePreprocessor&&)      = delete;
    virtual ~RobotStagePreprocessor() {}

    robot_kinematic::RobotKinematic::Ptr _robot_kinematic;
    robot_collision::RobotCollision::UPtr _robot_collision;
    std::vector<Eigen::Matrix<double, 4, 4>> _kinematic_chain;
    Eigen::Matrix<double, 6, -1> _jacobian;
    std::vector<Eigen::VectorXd> _vector_data;
    std::vector<Eigen::MatrixXd> _matrix_data;

    virtual void precompute(const Eigen::Ref<const Eigen::VectorXd>& input);

    void update();
    void set(bool static_obstacle, bool self_collision, bool roof_collision, bool ground_collision, bool plane_collision);
    bool isInitialized() const;

 protected:
    using ObstacleList = robot_obstacle::ObstacleList;

    bool _ground_collision = true;
    bool _roof_collision   = true;
    bool _self_collision   = true;
    bool _static_obstacles = true;
    bool _plane_collision  = true;
    bool _initialized      = false;
    int _dim               = 0;

    std::vector<robot_misc::Obstacle> _static_obstacle_list;
    std::vector<robot_misc::Plane> _plane_collision_list;
    ObstacleList _obstacle_manager;

    Eigen::VectorXd _input;
};

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot

#endif  // ROBOT_STAGE_PREPROCESSOR_H
