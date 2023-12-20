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

#include <mhp_robot/robot_trajectory_optimization/robot_stage_preprocessor.h>

namespace mhp_robot {
namespace robot_trajectory_optimization {

RobotStagePreprocessor::RobotStagePreprocessor(bool static_obstacle, bool self_collision, bool roof_collision, bool ground_collision,
                                               bool plane_collision, robot_collision::RobotCollision::UPtr robot_collision)
    : _robot_collision(std::move(robot_collision)),
      _ground_collision(ground_collision),
      _roof_collision(roof_collision),
      _self_collision(self_collision),
      _static_obstacles(static_obstacle),
      _plane_collision(plane_collision)
{
    _robot_kinematic = _robot_collision->getRobotKinematic();
    _dim             = _robot_kinematic->getRobotUtility().getJointsCount();

    _initialized = true;
}

void RobotStagePreprocessor::update()
{
    if (!_initialized) return;

    _vector_data.clear();

    if (_self_collision)
    {
        _vector_data.push_back(Eigen::VectorXd::Zero(_robot_collision->_self_collision_distance_dimension));
        _vector_data.push_back(Eigen::VectorXd::Zero(1));
    }
    else
    {
        _vector_data.push_back(Eigen::VectorXd::Zero(0));
        _vector_data.push_back(Eigen::VectorXd::Zero(0));
    }

    if (_static_obstacles)
    {
        _obstacle_manager._mutex.lock();
        _static_obstacle_list = _obstacle_manager._static_obstacles;
        _obstacle_manager._mutex.unlock();

        _vector_data.push_back(Eigen::VectorXd::Zero(_robot_collision->_obstacle_distance_dimension * _static_obstacle_list.size()));
        _vector_data.push_back(Eigen::VectorXd::Zero(_static_obstacle_list.size()));
    }
    else
    {
        _vector_data.push_back(Eigen::VectorXd::Zero(0));
        _vector_data.push_back(Eigen::VectorXd::Zero(0));
    }

    if (_ground_collision)
    {
        _vector_data.push_back(Eigen::VectorXd::Zero(_robot_collision->_plane_collision_distance_dimension));
        _vector_data.push_back(Eigen::VectorXd::Zero(1));
    }
    else
    {
        _vector_data.push_back(Eigen::VectorXd::Zero(0));
        _vector_data.push_back(Eigen::VectorXd::Zero(0));
    }

    if (_roof_collision)
    {
        _vector_data.push_back(Eigen::VectorXd::Zero(_robot_collision->_plane_collision_distance_dimension));
        _vector_data.push_back(Eigen::VectorXd::Zero(1));
    }
    else
    {
        _vector_data.push_back(Eigen::VectorXd::Zero(0));
        _vector_data.push_back(Eigen::VectorXd::Zero(0));
    }

    if (_plane_collision)
    {
        _obstacle_manager._mutex.lock();
        _plane_collision_list = _obstacle_manager._planes;
        _obstacle_manager._mutex.unlock();

        _vector_data.push_back(Eigen::VectorXd::Zero(_robot_collision->_plane_collision_distance_dimension * _plane_collision_list.size()));
        _vector_data.push_back(Eigen::VectorXd::Zero(_plane_collision_list.size()));
    }
    else
    {
        _vector_data.push_back(Eigen::VectorXd::Zero(0));
        _vector_data.push_back(Eigen::VectorXd::Zero(0));
    }

    _input = Eigen::VectorXd::Zero(_dim);
}

void RobotStagePreprocessor::precompute(const Eigen::Ref<const Eigen::VectorXd>& input)
{
    if (!_initialized) return;
    if (_input.isApprox(input)) return;

    _input = input;

    // Precompute kinematics
    _robot_collision->setJointState(_input);

    // Cache
    _kinematic_chain = _robot_kinematic->getForwardKinematicChain();
    _jacobian        = _robot_kinematic->getGeometricJacobian();

    // Put each distance typ in its own element
    Eigen::Ref<Eigen::VectorXd> self_collision_distances       = _vector_data[0];
    Eigen::Ref<Eigen::VectorXd> self_collision_min_distances   = _vector_data[1];
    Eigen::Ref<Eigen::VectorXd> static_obstacle_distances      = _vector_data[2];
    Eigen::Ref<Eigen::VectorXd> static_obstacles_min_distances = _vector_data[3];
    Eigen::Ref<Eigen::VectorXd> ground_collision_distances     = _vector_data[4];
    Eigen::Ref<Eigen::VectorXd> ground_collision_min_distances = _vector_data[5];
    Eigen::Ref<Eigen::VectorXd> roof_collision_distances       = _vector_data[6];
    Eigen::Ref<Eigen::VectorXd> roof_collision_min_distances   = _vector_data[7];
    Eigen::Ref<Eigen::VectorXd> plane_collision_distances      = _vector_data[8];
    Eigen::Ref<Eigen::VectorXd> plane_collision_min_distances  = _vector_data[9];

    // Precompute distances
    if (_self_collision)
    {
        _robot_collision->getSelfCollisionDistances(self_collision_distances);
        self_collision_min_distances(0) = self_collision_distances.minCoeff();
    }

    if (_ground_collision)
    {
        _robot_collision->getGroundCollisionDistances(ground_collision_distances);
        ground_collision_min_distances(0) = ground_collision_distances.minCoeff();
    }

    if (_roof_collision)
    {
        _robot_collision->getRoofCollisionDistances(roof_collision_distances);
        roof_collision_min_distances(0) = roof_collision_distances.minCoeff();
    }

    if (_static_obstacles)
    {
        for (int i = 0; i < _static_obstacle_list.size(); ++i)
        {
            Eigen::Ref<Eigen::VectorXd> distance_mask =
                static_obstacle_distances.segment(i * _robot_collision->_obstacle_distance_dimension, _robot_collision->_obstacle_distance_dimension);

            _robot_collision->getObstacleDistances(_static_obstacle_list[i], distance_mask);
            static_obstacles_min_distances(i) = distance_mask.minCoeff();
        }
    }

    if (_plane_collision)
    {
        for (int i = 0; i < _plane_collision_list.size(); ++i)
        {
            Eigen::Ref<Eigen::VectorXd> distance_mask = plane_collision_distances.segment(i * _robot_collision->_plane_collision_distance_dimension,
                                                                                          _robot_collision->_plane_collision_distance_dimension);

            _robot_collision->getPlaneCollisionDistances(_plane_collision_list[i], distance_mask);
            plane_collision_min_distances(i) = distance_mask.minCoeff();
        }
    }
}

void RobotStagePreprocessor::set(bool static_obstacle, bool self_collision, bool roof_collision, bool ground_collision, bool plane_collision)
{
    _static_obstacles = static_obstacle;
    _ground_collision = ground_collision;
    _roof_collision   = roof_collision;
    _self_collision   = self_collision;
    _plane_collision  = plane_collision;
}

bool RobotStagePreprocessor::isInitialized() const { return _initialized; }

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot
