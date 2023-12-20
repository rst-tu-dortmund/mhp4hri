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

#include <mhp_robot/robot_trajectory_optimization/robot_collision_constraint.h>

namespace mhp_robot {
namespace robot_trajectory_optimization {

bool RobotCollisionConstraint::initialize(RobotStagePreprocessor::Ptr preprocessor, RobotCollision::UPtr robot_collision)
{
    _robot_collision = std::move(robot_collision);

    if (preprocessor)
    {
        _preprocessor = preprocessor;
    }
    else
    {
        if (_use_preprocessor) ROS_WARN("RobotCollisionConstraint: No preprocessor provided. No preprocessor is used!");
        _use_preprocessor = false;
    }

    _initialized = true;

    return true;
}

void RobotCollisionConstraint::compute(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost)
{
    int ns = 0;
    int np = 0;
    if (_use_preprocessor)
    {
        _preprocessor->precompute(x_k);
        ns = (int)_preprocessor->_vector_data[3].rows();
        np = (int)_preprocessor->_vector_data[9].rows();
    }
    else
    {
        _robot_collision->setJointState(x_k);
        ns = _static_obstacle_list.size();
        np = _plane_collision_list.size();
    }

    int nd = _dynamic_obstacle_list.size();
    int nh = _human_obstacle_list.size();

    double t = (_obstacle_prediction ? k * _dt : 0.0);
    _planning_delay._mutex.lock();
    t += (_prediction_offset ? _planning_delay._delay : 0.0);
    _planning_delay._mutex.unlock();
    int offset = 0;

    if (_self_collision)
    {
        if (_min_distances)
        {
            if (_use_preprocessor)
            {
                cost(offset) = _self_collision_threshold - _preprocessor->_vector_data[1](0);  // use precomputed distances
            }
            else
            {
                cost(offset) = _self_collision_threshold - _robot_collision->getMinSelfCollisionDistance();
            }
            offset += 1;
        }
        else
        {
            Eigen::Ref<Eigen::VectorXd> distance_mask = cost.segment(offset, _robot_collision->_self_collision_distance_dimension);

            if (_use_preprocessor)
            {
                distance_mask = _preprocessor->_vector_data[0];  // use precomputed distances
            }
            else
            {
                _robot_collision->getSelfCollisionDistances(distance_mask);
            }

            distance_mask = _self_collision_threshold - distance_mask.array();

            offset += _robot_collision->_self_collision_distance_dimension;
        }
    }

    if (_ground_collision)
    {
        if (_min_distances)
        {
            if (_use_preprocessor)
            {
                cost(offset) = _ground_collision_threshold - _preprocessor->_vector_data[5](0);  // use precomputed distances
            }
            else
            {
                cost(offset) = _ground_collision_threshold - _robot_collision->getMinGroundCollisionDistance();
            }
            offset += 1;
        }
        else
        {
            Eigen::Ref<Eigen::VectorXd> distance_mask = cost.segment(offset, _robot_collision->_plane_collision_distance_dimension);

            if (_use_preprocessor)
            {
                distance_mask = _preprocessor->_vector_data[4];  // use precomputed distances
            }
            else
            {
                _robot_collision->getGroundCollisionDistances(distance_mask);
            }

            distance_mask = _ground_collision_threshold - distance_mask.array();

            offset += _robot_collision->_plane_collision_distance_dimension;
        }
    }

    if (_roof_collision)
    {
        if (_min_distances)
        {
            if (_use_preprocessor)
            {
                cost(offset) = _roof_collision_threshold - _preprocessor->_vector_data[7](0);  // use precomputed distances
            }
            else
            {
                cost(offset) = _roof_collision_threshold - _robot_collision->getMinRoofCollisionDistance();
            }
            offset += 1;
        }
        else
        {
            Eigen::Ref<Eigen::VectorXd> distance_mask = cost.segment(offset, _robot_collision->_plane_collision_distance_dimension);

            if (_use_preprocessor)
            {
                distance_mask = _preprocessor->_vector_data[6];  // use precomputed distances
            }
            else
            {
                _robot_collision->getRoofCollisionDistances(distance_mask);
            }

            distance_mask = _roof_collision_threshold - distance_mask.array();

            offset += _robot_collision->_plane_collision_distance_dimension;
        }
    }

    if (_static_obstacles)
    {
        for (int i = 0; i < ns; ++i)
        {
            if (_min_distances)
            {
                if (_use_preprocessor)
                {
                    cost(offset) = _static_obstacle_threshold - _preprocessor->_vector_data[3](i);  // use precomputed distances
                }
                else
                {
                    cost(offset) = _static_obstacle_threshold - _robot_collision->getMinObstacleDistance(_static_obstacle_list[i]);
                }
                offset += 1;
            }
            else
            {
                Eigen::Ref<Eigen::VectorXd> distance_mask = cost.segment(offset, _robot_collision->_obstacle_distance_dimension);

                if (_use_preprocessor)
                {
                    distance_mask =
                        _preprocessor->_vector_data[2].segment(i * _robot_collision->_obstacle_distance_dimension,
                                                               _robot_collision->_obstacle_distance_dimension);  // use precomputed distances
                }
                else
                {
                    _robot_collision->getObstacleDistances(_static_obstacle_list[i], distance_mask);
                }

                distance_mask = _static_obstacle_threshold - distance_mask.array();

                offset += _robot_collision->_obstacle_distance_dimension;
            }
        }
    }

    if (_dynamic_obstacles)
    {
        for (int i = 0; i < nd; ++i)
        {
            if (_min_distances)
            {
                if (_use_preprocessor)
                {
                    cost(offset) =
                        _dynamic_obstacle_threshold - _preprocessor->_robot_collision->getMinObstacleDistance(_dynamic_obstacle_list[i], t);
                }
                else
                {
                    cost(offset) = _dynamic_obstacle_threshold - _robot_collision->getMinObstacleDistance(_dynamic_obstacle_list[i], t);
                }
                offset += 1;
            }
            else
            {
                Eigen::Ref<Eigen::VectorXd> distance_mask = cost.segment(offset, _robot_collision->_obstacle_distance_dimension);

                if (_use_preprocessor)
                {
                    _preprocessor->_robot_collision->getObstacleDistances(_dynamic_obstacle_list[i], distance_mask, t);
                }
                else
                {
                    _robot_collision->getObstacleDistances(_dynamic_obstacle_list[i], distance_mask, t);
                }

                distance_mask = _dynamic_obstacle_threshold - distance_mask.array();

                offset += _robot_collision->_obstacle_distance_dimension;
            }
        }
    }

    if (_human_head_collision || _human_trunk_collision || _human_limbs_collision)
    {
        double head_threshold                                      = 0.0;
        double limbs_threshold                                     = 0.0;
        double trunk_threshold                                     = 0.0;
        std::tie(head_threshold, trunk_threshold, limbs_threshold) = _human_obstacle_thresholds;

        for (int i = 0; i < nh; ++i)
        {
            if (_min_distances)
            {
                if (_use_preprocessor)
                {
                    if (_human_head_collision)
                    {
                        cost(offset) = _preprocessor->_robot_collision->getMinHumanHeadDistance(_human_obstacle_list[i], t);
                        cost(offset) = head_threshold - cost(offset);
                        offset += 1;
                    }
                    if (_human_trunk_collision)
                    {
                        _preprocessor->_robot_collision->getMinHumanTrunkDistance(_human_obstacle_list[i], cost.segment<2>(offset), t);
                        cost.segment<2>(offset) = trunk_threshold - cost.segment<2>(offset).array();
                        offset += 2;
                    }
                    if (_human_limbs_collision)
                    {
                        _preprocessor->_robot_collision->getMinHumanLimbsDistance(_human_obstacle_list[i], cost.segment<4>(offset), t);
                        cost.segment<4>(offset) = limbs_threshold - cost.segment<4>(offset).array();
                        offset += 4;
                    }
                }
                else
                {
                    if (_ms_planner_mode)
                    {
                        if (_planner_id == 1)
                        {
                            if (_human_head_collision)
                            {
                                cost(offset) = _robot_collision->getMinHumanHeadDistance(_human_obstacle_list[i], t);
                                cost(offset) = head_threshold - cost(offset);
                                offset += 1;
                            }
                            if (_human_trunk_collision)
                            {
                                _robot_collision->getMinHumanTrunkDistance(_human_obstacle_list[i], cost.segment<2>(offset), t);
                                cost.segment<2>(offset) = trunk_threshold - cost.segment<2>(offset).array();
                                offset += 2;
                            }
                            if (_human_limbs_collision)
                            {
                                _robot_collision->getMinHumanLimbsDistance(_human_obstacle_list[i], cost.segment<4>(offset), t);
                                cost.segment<4>(offset) = limbs_threshold - cost.segment<4>(offset).array();
                                offset += 4;
                            }
                        }
                        else
                        {
                            if (_human_head_collision)
                            {
                                cost(offset) = _robot_collision->getMinHumanHeadDistanceUncertaintyInstance(
                                    _human_obstacle_list[i], _human_obstacle_list[i]._ms_mapping[_planner_id - 2], t);
                                cost(offset) = head_threshold - cost(offset);
                                offset += 1;
                            }
                            if (_human_trunk_collision)
                            {
                                _robot_collision->getMinHumanTrunkDistanceUncertaintyInstance(
                                    _human_obstacle_list[i], cost.segment<2>(offset), _human_obstacle_list[i]._ms_mapping[_planner_id - 2], t);
                                cost.segment<2>(offset) = trunk_threshold - cost.segment<2>(offset).array();
                                offset += 2;
                            }
                            if (_human_limbs_collision)
                            {
                                _robot_collision->getMinHumanLimbsDistanceUncertaintyInstance(
                                    _human_obstacle_list[i], cost.segment<4>(offset), _human_obstacle_list[i]._ms_mapping[_planner_id - 2], t);
                                cost.segment<4>(offset) = limbs_threshold - cost.segment<4>(offset).array();
                                offset += 4;
                            }
                        }
                    }
                    else
                    {
                        if (_human_head_collision)
                        {
                            cost(offset) = _robot_collision->getMinHumanHeadDistance(_human_obstacle_list[i], t);
                            cost(offset) = head_threshold - cost(offset);
                            offset += 1;
                        }
                        if (_human_trunk_collision)
                        {
                            _robot_collision->getMinHumanTrunkDistance(_human_obstacle_list[i], cost.segment<2>(offset), t);
                            cost.segment<2>(offset) = trunk_threshold - cost.segment<2>(offset).array();
                            offset += 2;
                        }
                        if (_human_limbs_collision)
                        {
                            _robot_collision->getMinHumanLimbsDistance(_human_obstacle_list[i], cost.segment<4>(offset), t);
                            cost.segment<4>(offset) = limbs_threshold - cost.segment<4>(offset).array();
                            offset += 4;
                        }
                    }
                }
            }
            else
            {
                if (_use_preprocessor)
                {
                    if (_human_head_collision)
                    {
                        _preprocessor->_robot_collision->getHumanHeadDistances(
                            _human_obstacle_list[i], cost.segment(offset, _robot_collision->_human_collision_distance_dimension), t);

                        cost.segment(offset, _robot_collision->_human_collision_distance_dimension) =
                            head_threshold - cost.segment(offset, _robot_collision->_human_collision_distance_dimension).array();

                        offset += _robot_collision->_human_collision_distance_dimension;
                    }
                    if (_human_trunk_collision)
                    {
                        _preprocessor->_robot_collision->getHumanTrunkDistances(
                            _human_obstacle_list[i], cost.segment(offset, 2 * _robot_collision->_human_collision_distance_dimension), t);

                        cost.segment(offset, 2 * _robot_collision->_human_collision_distance_dimension) =
                            trunk_threshold - cost.segment(offset, 2 * _robot_collision->_human_collision_distance_dimension).array();

                        offset += 2 * _robot_collision->_human_collision_distance_dimension;
                    }
                    if (_human_limbs_collision)
                    {
                        _preprocessor->_robot_collision->getHumanLimbsDistances(
                            _human_obstacle_list[i], cost.segment(offset, 4 * _robot_collision->_human_collision_distance_dimension), t);

                        cost.segment(offset, 4 * _robot_collision->_human_collision_distance_dimension) =
                            limbs_threshold - cost.segment(offset, 4 * _robot_collision->_human_collision_distance_dimension).array();

                        offset += 4 * _robot_collision->_human_collision_distance_dimension;
                    }
                }
                else
                {
                    if (_ms_planner_mode)
                    {
                        if (_planner_id == 1)
                        {
                            if (_human_head_collision)
                            {
                                _robot_collision->getHumanHeadDistances(
                                    _human_obstacle_list[i], cost.segment(offset, _robot_collision->_human_collision_distance_dimension), t);

                                cost.segment(offset, _robot_collision->_human_collision_distance_dimension) =
                                    head_threshold - cost.segment(offset, _robot_collision->_human_collision_distance_dimension).array();

                                offset += _robot_collision->_human_collision_distance_dimension;
                            }
                            if (_human_trunk_collision)
                            {
                                _robot_collision->getHumanTrunkDistances(
                                    _human_obstacle_list[i], cost.segment(offset, 2 * _robot_collision->_human_collision_distance_dimension), t);

                                cost.segment(offset, 2 * _robot_collision->_human_collision_distance_dimension) =
                                    trunk_threshold - cost.segment(offset, 2 * _robot_collision->_human_collision_distance_dimension).array();

                                offset += 2 * _robot_collision->_human_collision_distance_dimension;
                            }
                            if (_human_limbs_collision)
                            {
                                _robot_collision->getHumanLimbsDistances(
                                    _human_obstacle_list[i], cost.segment(offset, 4 * _robot_collision->_human_collision_distance_dimension), t);

                                cost.segment(offset, 4 * _robot_collision->_human_collision_distance_dimension) =
                                    limbs_threshold - cost.segment(offset, 4 * _robot_collision->_human_collision_distance_dimension).array();

                                offset += 4 * _robot_collision->_human_collision_distance_dimension;
                            }
                        }
                        else
                        {
                            if (_human_head_collision)
                            {
                                _robot_collision->getHumanHeadDistancesUncertaintyInstance(
                                    _human_obstacle_list[i], cost.segment(offset, _robot_collision->_human_collision_distance_dimension),
                                    _human_obstacle_list[i]._ms_mapping[_planner_id - 2], t);

                                cost.segment(offset, _robot_collision->_human_collision_distance_dimension) =
                                    head_threshold - cost.segment(offset, _robot_collision->_human_collision_distance_dimension).array();

                                offset += _robot_collision->_human_collision_distance_dimension;
                            }
                            if (_human_trunk_collision)
                            {
                                _robot_collision->getHumanTrunkDistancesUncertaintyInstance(
                                    _human_obstacle_list[i], cost.segment(offset, 2 * _robot_collision->_human_collision_distance_dimension),
                                    _human_obstacle_list[i]._ms_mapping[_planner_id - 2], t);

                                cost.segment(offset, 2 * _robot_collision->_human_collision_distance_dimension) =
                                    trunk_threshold - cost.segment(offset, 2 * _robot_collision->_human_collision_distance_dimension).array();

                                offset += 2 * _robot_collision->_human_collision_distance_dimension;
                            }
                            if (_human_limbs_collision)
                            {
                                _robot_collision->getHumanLimbsDistancesUncertaintyInstance(
                                    _human_obstacle_list[i], cost.segment(offset, 4 * _robot_collision->_human_collision_distance_dimension),
                                    _human_obstacle_list[i]._ms_mapping[_planner_id - 2], t);

                                cost.segment(offset, 4 * _robot_collision->_human_collision_distance_dimension) =
                                    limbs_threshold - cost.segment(offset, 4 * _robot_collision->_human_collision_distance_dimension).array();

                                offset += 4 * _robot_collision->_human_collision_distance_dimension;
                            }
                        }
                    }
                    else
                    {
                        if (_human_head_collision)
                        {
                            _robot_collision->getHumanHeadDistances(_human_obstacle_list[i],
                                                                    cost.segment(offset, _robot_collision->_human_collision_distance_dimension), t);

                            cost.segment(offset, _robot_collision->_human_collision_distance_dimension) =
                                head_threshold - cost.segment(offset, _robot_collision->_human_collision_distance_dimension).array();

                            offset += _robot_collision->_human_collision_distance_dimension;
                        }
                        if (_human_trunk_collision)
                        {
                            _robot_collision->getHumanTrunkDistances(
                                _human_obstacle_list[i], cost.segment(offset, 2 * _robot_collision->_human_collision_distance_dimension), t);

                            cost.segment(offset, 2 * _robot_collision->_human_collision_distance_dimension) =
                                trunk_threshold - cost.segment(offset, 2 * _robot_collision->_human_collision_distance_dimension).array();

                            offset += 2 * _robot_collision->_human_collision_distance_dimension;
                        }
                        if (_human_limbs_collision)
                        {
                            _robot_collision->getHumanLimbsDistances(
                                _human_obstacle_list[i], cost.segment(offset, 4 * _robot_collision->_human_collision_distance_dimension), t);

                            cost.segment(offset, 4 * _robot_collision->_human_collision_distance_dimension) =
                                limbs_threshold - cost.segment(offset, 4 * _robot_collision->_human_collision_distance_dimension).array();

                            offset += 4 * _robot_collision->_human_collision_distance_dimension;
                        }
                    }
                }
            }
        }
    }

    if (_plane_collision)
    {
        for (int i = 0; i < np; ++i)
        {
            if (_min_distances)
            {
                if (_use_preprocessor)
                {
                    cost(offset) = _plane_collision_threshold - _preprocessor->_vector_data[9](i);  // use precomputed distances
                }
                else
                {
                    cost(offset) = _plane_collision_threshold - _robot_collision->getMinPlaneCollisionDistance(_plane_collision_list[i]);
                }
                offset += 1;
            }
            else
            {
                Eigen::Ref<Eigen::VectorXd> distance_mask = cost.segment(offset, _robot_collision->_plane_collision_distance_dimension);

                if (_use_preprocessor)
                {
                    distance_mask =
                        _preprocessor->_vector_data[8].segment(i * _robot_collision->_plane_collision_distance_dimension,
                                                               _robot_collision->_plane_collision_distance_dimension);  // use precomputed distances
                }
                else
                {
                    _robot_collision->getPlaneCollisionDistances(_plane_collision_list[i], distance_mask);
                }

                distance_mask = _plane_collision_threshold - distance_mask.array();

                offset += _robot_collision->_plane_collision_distance_dimension;
            }
        }
    }
}

int RobotCollisionConstraint::getDimension() const { return _dimension; }

void RobotCollisionConstraint::update(double dt)
{
    _dimension = 0;
    _dt        = dt;

    //    std::cout << "In Robot collision constraint fpr planner: " << _planner_id << std::endl;
    if (_dynamic_obstacles)
    {
        _obstacle_manager._mutex.lock();
        _dynamic_obstacle_list = _obstacle_manager._dynamic_obstacles;
        _obstacle_manager._mutex.unlock();

        if (_min_distances)
        {
            _dimension += _dynamic_obstacle_list.size();
        }
        else
        {
            _dimension += _robot_collision->_obstacle_distance_dimension * _dynamic_obstacle_list.size();
        }
    }

    if (_human_head_collision || _human_trunk_collision || _human_limbs_collision)
    {
        _obstacle_manager._mutex.lock();
        _human_obstacle_list = _obstacle_manager._humans;
        _obstacle_manager._mutex.unlock();

        if (_min_distances)
        {
            if (_human_head_collision) _dimension += _human_obstacle_list.size();
            if (_human_trunk_collision) _dimension += 2 * _human_obstacle_list.size();
            if (_human_limbs_collision) _dimension += 4 * _human_obstacle_list.size();
        }
        else
        {
            if (_human_head_collision) _dimension += _human_obstacle_list.size() * _robot_collision->_human_collision_distance_dimension;
            if (_human_trunk_collision) _dimension += 2 * _human_obstacle_list.size() * _robot_collision->_human_collision_distance_dimension;
            if (_human_limbs_collision) _dimension += 4 * _human_obstacle_list.size() * _robot_collision->_human_collision_distance_dimension;
        }
    }

    if (!_use_preprocessor)
    {
        if (_static_obstacles)
        {
            _obstacle_manager._mutex.lock();
            _static_obstacle_list = _obstacle_manager._static_obstacles;
            _obstacle_manager._mutex.unlock();

            if (_min_distances)
            {
                _dimension += _static_obstacle_list.size();
            }
            else
            {
                _dimension += _robot_collision->_obstacle_distance_dimension * _static_obstacle_list.size();
            }
        }

        if (_ground_collision)
        {
            if (_min_distances)
            {
                _dimension += 1;
            }
            else
            {
                _dimension += _robot_collision->_plane_collision_distance_dimension;
            }
        }

        if (_roof_collision)
        {
            if (_min_distances)
            {
                _dimension += 1;
            }
            else
            {
                _dimension += _robot_collision->_plane_collision_distance_dimension;
            }
        }

        if (_self_collision)
        {
            if (_min_distances)
            {
                _dimension += 1;
            }
            else
            {
                _dimension += _robot_collision->_self_collision_distance_dimension;
            }
        }

        if (_plane_collision)
        {
            _obstacle_manager._mutex.lock();
            _plane_collision_list = _obstacle_manager._planes;
            _obstacle_manager._mutex.unlock();

            if (_min_distances)
            {
                _dimension += _plane_collision_list.size();
            }
            else
            {
                _dimension += _robot_collision->_plane_collision_distance_dimension * _plane_collision_list.size();
            }
        }
    }
    else
    {
        if (_min_distances)
        {
            if (_self_collision) _dimension = _preprocessor->_vector_data[1].rows();
            if (_static_obstacles) _dimension = _preprocessor->_vector_data[3].rows();
            if (_ground_collision) _dimension = _preprocessor->_vector_data[5].rows();
            if (_roof_collision) _dimension = _preprocessor->_vector_data[7].rows();
            if (_plane_collision) _dimension = _preprocessor->_vector_data[9].rows();
        }
        else
        {
            if (_self_collision) _dimension = _preprocessor->_vector_data[0].rows();
            if (_static_obstacles) _dimension = _preprocessor->_vector_data[2].rows();
            if (_ground_collision) _dimension = _preprocessor->_vector_data[4].rows();
            if (_roof_collision) _dimension = _preprocessor->_vector_data[6].rows();
            if (_plane_collision) _dimension = _preprocessor->_vector_data[8].rows();
        }
    }
}

bool RobotCollisionConstraint::isInitialized() const { return _initialized; }

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot
