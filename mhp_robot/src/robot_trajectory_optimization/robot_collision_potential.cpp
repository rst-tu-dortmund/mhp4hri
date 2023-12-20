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

#include <mhp_robot/robot_trajectory_optimization/robot_collision_potential.h>

namespace mhp_robot {
namespace robot_trajectory_optimization {

void RobotCollisionPotential::computeGradient(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> dx)
{
    Eigen::VectorXd diff = Eigen::VectorXd::Zero(x_k.size());
    for (int i = 0; i < x_k.size(); ++i)
    {
        diff(i) = _eps;
        dx(i)   = (computePotentials(k, x_k + diff) - computePotentials(k, x_k - diff)) / (2 * _eps);
        diff(i) = 0.0;
    }
}

void RobotCollisionPotential::computeHessian(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::MatrixXd> dxdx)
{
    int n                = x_k.size();
    Eigen::VectorXd diff = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd ldx(n), rdx(n);
    for (int i = 0; i < n; ++i)
    {
        diff(i) = _eps;
        computeGradient(k, x_k + diff, rdx);
        computeGradient(k, x_k - diff, ldx);
        dxdx.col(i) = (rdx - ldx) / (2 * _eps);
        diff(i)     = 0.0;
    }
}

bool RobotCollisionPotential::initialize(RobotStagePreprocessor::Ptr preprocessor, RobotCollision::UPtr robot_collision)
{
    _robot_collision = std::move(robot_collision);

    if (preprocessor)
    {
        _preprocessor = preprocessor;
    }
    else
    {
        if (_use_preprocessor) ROS_WARN("RobotCollisionPotential: No preprocessor provided. No preprocessor is used!");
        _use_preprocessor = false;
    }

    _initialized = true;

    return true;
}

bool RobotCollisionPotential::update(double dt)
{
    _dt = dt;

    _ground_collision_dimension                  = 0;
    _roof_collision_dimension                    = 0;
    _self_collision_dimension                    = 0;
    _static_obstacle_dimension                   = 0;
    _dynamic_obstacle_dimension                  = 0;
    _plane_collision_dimension                   = 0;
    _human_head_collision_dimension              = 0;
    _human_trunk_collision_dimension             = 0;
    _human_limbs_collision_dimension             = 0;
    _human_uncertainty_head_collision_dimension  = 0;
    _human_uncertainty_trunk_collision_dimension = 0;
    _human_uncertainty_limbs_collision_dimension = 0;

    if (_dynamic_obstacles)
    {
        _obstacle_manager._mutex.lock();
        _dynamic_obstacle_list = _obstacle_manager._dynamic_obstacles;
        _obstacle_manager._mutex.unlock();

        if (_min_distances)
        {
            _dynamic_obstacle_dimension = _dynamic_obstacle_list.size();
        }
        else
        {
            _dynamic_obstacle_dimension = _robot_collision->_obstacle_distance_dimension * _dynamic_obstacle_list.size();
        }
    }

    if (_human_head_collision || _human_trunk_collision || _human_limbs_collision)
    {
        _obstacle_manager._mutex.lock();
        _human_obstacle_list = _obstacle_manager._humans;
        _obstacle_manager._mutex.unlock();

        if (_uncertainty_estimation_human && _human_obstacle_list.size() == 1)
        {
            if (_min_distances)
            {
                // Default distances dimension update
                if (_human_head_collision) _human_head_collision_dimension = _human_obstacle_list.size();
                if (_human_trunk_collision) _human_trunk_collision_dimension = 2 * _human_obstacle_list.size();
                if (_human_limbs_collision) _human_limbs_collision_dimension = 4 * _human_obstacle_list.size();

                // Radius Increase Dimensions (same as normal)
                if (_human_head_collision && _uncertainty_mode == RadiusIncrease)
                    _human_uncertainty_head_collision_dimension = _human_obstacle_list.size();
                if (_human_trunk_collision && _uncertainty_mode == RadiusIncrease)
                    _human_uncertainty_trunk_collision_dimension = 2 * _human_obstacle_list.size();
                if (_human_limbs_collision && _uncertainty_mode == RadiusIncrease)
                    _human_uncertainty_limbs_collision_dimension = 4 * _human_obstacle_list.size();

                // Uncertainty distances dimension update for skeleton splitting (radius increase reuses default dimensions because remains same)
                if (_human_head_collision && (_human_obstacle_list.at(0).getNumberUncertaintyHead() > 0) && _uncertainty_mode == SkeletonSplitting)
                    _human_uncertainty_head_collision_dimension = _human_obstacle_list.size();
                if (_human_trunk_collision &&
                    (_human_obstacle_list.at(0).getNumberUncertaintyNeck() > 0 || _human_obstacle_list.at(0).getNumberUncertaintyLeg() > 0) &&
                    _uncertainty_mode == SkeletonSplitting)
                    _human_uncertainty_trunk_collision_dimension = 2 * _human_obstacle_list.size();
                if (_human_limbs_collision &&
                    (_human_obstacle_list.at(0).getNumberUncertaintyLUArm() > 0 || _human_obstacle_list.at(0).getNumberUncertaintyLFArm() > 0 ||
                     _human_obstacle_list.at(0).getNumberUncertaintyRUArm() > 0 || _human_obstacle_list.at(0).getNumberUncertaintyRFArm() > 0) &&
                    _uncertainty_mode == SkeletonSplitting)
                    _human_uncertainty_limbs_collision_dimension = 4 * _human_obstacle_list.size();
                //                std::cout << "Human dimensions Min Distances: " << _human_head_collision_dimension << " " <<
                //                _human_trunk_collision_dimension << " "
                //                          << _human_limbs_collision_dimension << " " << _human_uncertainty_head_collision_dimension << " "
                //                          << _human_uncertainty_trunk_collision_dimension << " " << _human_uncertainty_limbs_collision_dimension <<
                //                          std::endl;

                // If we use multistage planner, then we don't consider Uncertainties for each planner but each uncertainty instance as
                // initialization. Other uncertaintuies are neglected then.
                if (_ms_planner_mode)
                {
                    _human_uncertainty_head_collision_dimension  = 0;
                    _human_uncertainty_trunk_collision_dimension = 0;
                    _human_uncertainty_limbs_collision_dimension = 0;
                }
            }
            else
            {
                // Default distances dimension update
                if (_human_head_collision)
                    _human_head_collision_dimension = _human_obstacle_list.size() * _robot_collision->_obstacle_distance_dimension;
                if (_human_trunk_collision)
                    _human_trunk_collision_dimension = 2 * _human_obstacle_list.size() * _robot_collision->_obstacle_distance_dimension;
                if (_human_limbs_collision)
                    _human_limbs_collision_dimension = 4 * _human_obstacle_list.size() * _robot_collision->_obstacle_distance_dimension;

                // Radius Increase
                if (_human_head_collision && _uncertainty_mode == RadiusIncrease)
                    _human_uncertainty_head_collision_dimension = _human_obstacle_list.size() * _robot_collision->_obstacle_distance_dimension;
                if (_human_trunk_collision && _uncertainty_mode == RadiusIncrease)
                    _human_uncertainty_trunk_collision_dimension = 2 * _human_obstacle_list.size() * _robot_collision->_obstacle_distance_dimension;
                if (_human_limbs_collision && _uncertainty_mode == RadiusIncrease)
                    _human_uncertainty_limbs_collision_dimension = 4 * _human_obstacle_list.size() * _robot_collision->_obstacle_distance_dimension;

                // Uncertainty distances dimension update for skeleton splitting (radius increase reuses default dimensions because remains same)
                if (_human_head_collision && _uncertainty_mode == SkeletonSplitting)
                    _human_uncertainty_head_collision_dimension =
                        _human_obstacle_list.at(0).getNumberUncertaintyHead() * _robot_collision->_obstacle_distance_dimension;
                if (_human_trunk_collision && _uncertainty_mode == SkeletonSplitting)
                    _human_uncertainty_trunk_collision_dimension =
                        (_human_obstacle_list.at(0).getNumberUncertaintyNeck() + _human_obstacle_list.at(0).getNumberUncertaintyLeg()) *
                        _robot_collision->_obstacle_distance_dimension;
                if (_human_limbs_collision && _uncertainty_mode == SkeletonSplitting)
                    _human_uncertainty_limbs_collision_dimension =
                        (_human_obstacle_list.at(0).getNumberUncertaintyLUArm() + _human_obstacle_list.at(0).getNumberUncertaintyLFArm() +
                         _human_obstacle_list.at(0).getNumberUncertaintyRUArm() + _human_obstacle_list.at(0).getNumberUncertaintyRFArm()) *
                        _robot_collision->_obstacle_distance_dimension;

                //                if (_human_obstacle_list.at(0).getNumberUncertaintyLUArm() > 0)
                //                {
                //                    for (int i = 0; i < _human_obstacle_list.at(0).getNumberUncertaintyLUArm(); ++i)
                //                    {

                //                        std::cout << "Uncertainty Instance LUARM: "
                //                                  <<
                //                                  _human_obstacle_list.at(0).getBodyPart("LUArm").uncertainty_states.at(i).getUncertaintyInstance()
                //                                  << " at unc. number: " << i << std::endl;
                //                    }
                //                }

                // If we use multistage planner, then we don't consider Uncertainties for each planner but each uncertainty instance as
                // initialization. Other uncertaintuies are neglected then.
                if (_ms_planner_mode)
                {
                    _human_uncertainty_head_collision_dimension  = 0;
                    _human_uncertainty_trunk_collision_dimension = 0;
                    _human_uncertainty_limbs_collision_dimension = 0;
                }

                //                std::cout << "Human dimensions all distances: " << _human_head_collision_dimension << " " <<
                //                _human_trunk_collision_dimension << " "
                //                          << _human_limbs_collision_dimension << " " << _human_uncertainty_head_collision_dimension << " "
                //                          << _human_uncertainty_trunk_collision_dimension << " " << _human_uncertainty_limbs_collision_dimension
                //                          << " with ms_planner_mode: " << _ms_planner_mode << std::endl;
            }
        }
        else
        {
            if (_ms_planner_mode) ROS_ERROR("RobotCollisionPotential: Set uncertainty mode to true and skeleton splitting for MS-planners");

            if (_min_distances)
            {
                if (_human_head_collision) _human_head_collision_dimension = _human_obstacle_list.size();
                if (_human_trunk_collision) _human_trunk_collision_dimension = 2 * _human_obstacle_list.size();
                if (_human_limbs_collision) _human_limbs_collision_dimension = 4 * _human_obstacle_list.size();
            }
            else
            {
                if (_human_head_collision)
                    _human_head_collision_dimension = _human_obstacle_list.size() * _robot_collision->_obstacle_distance_dimension;
                if (_human_trunk_collision)
                    _human_trunk_collision_dimension = 2 * _human_obstacle_list.size() * _robot_collision->_obstacle_distance_dimension;
                if (_human_limbs_collision)
                    _human_limbs_collision_dimension = 4 * _human_obstacle_list.size() * _robot_collision->_obstacle_distance_dimension;
            }
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
                _static_obstacle_dimension = _static_obstacle_list.size();
            }
            else
            {
                _static_obstacle_dimension = _robot_collision->_obstacle_distance_dimension * _static_obstacle_list.size();
            }
        }

        if (_self_collision)
        {
            if (_min_distances)
            {
                _self_collision_dimension = 1;
            }
            else
            {
                _self_collision_dimension = _robot_collision->_self_collision_distance_dimension;
            }
        }

        if (_ground_collision)
        {
            if (_min_distances)
            {
                _ground_collision_dimension = 1;
            }
            else
            {
                _ground_collision_dimension = _robot_collision->_plane_collision_distance_dimension;
            }
        }

        if (_roof_collision)
        {
            if (_min_distances)
            {
                _roof_collision_dimension = 1;
            }
            else
            {
                _roof_collision_dimension = _robot_collision->_plane_collision_distance_dimension;
            }
        }

        if (_plane_collision)
        {
            _obstacle_manager._mutex.lock();
            _plane_obstacle_list = _obstacle_manager._planes;
            _obstacle_manager._mutex.unlock();

            if (_min_distances)
            {
                _plane_collision_dimension = _plane_obstacle_list.size();
            }
            else
            {
                _plane_collision_dimension = _robot_collision->_plane_collision_distance_dimension * _plane_obstacle_list.size();
            }
        }
    }
    else
    {
        if (_min_distances)
        {
            if (_self_collision) _self_collision_dimension = _preprocessor->_vector_data[1].rows();
            if (_static_obstacles) _static_obstacle_dimension = _preprocessor->_vector_data[3].rows();
            if (_ground_collision) _ground_collision_dimension = _preprocessor->_vector_data[5].rows();
            if (_roof_collision) _roof_collision_dimension = _preprocessor->_vector_data[7].rows();
            if (_plane_collision) _plane_collision_dimension = _preprocessor->_vector_data[9].rows();
        }
        else
        {
            if (_self_collision) _self_collision_dimension = _preprocessor->_vector_data[0].rows();
            if (_static_obstacles) _static_obstacle_dimension = _preprocessor->_vector_data[2].rows();
            if (_ground_collision) _ground_collision_dimension = _preprocessor->_vector_data[4].rows();
            if (_roof_collision) _roof_collision_dimension = _preprocessor->_vector_data[6].rows();
            if (_plane_collision) _plane_collision_dimension = _preprocessor->_vector_data[8].rows();
        }
    }
    return false;
}

bool RobotCollisionPotential::isInitialized() const { return _initialized; }

void RobotCollisionPotential::computeDistances(const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::ArrayXd> rc,
                                               Eigen::Ref<Eigen::ArrayXd> gc, Eigen::Ref<Eigen::ArrayXd> sc, Eigen::Ref<Eigen::ArrayXd> soc,
                                               Eigen::Ref<Eigen::ArrayXd> doc, Eigen::Ref<Eigen::ArrayXd> poc, Eigen::Ref<Eigen::ArrayXd> hhc,
                                               Eigen::Ref<Eigen::ArrayXd> htc, Eigen::Ref<Eigen::ArrayXd> hlc, Eigen::Ref<Eigen::ArrayXd> huh,
                                               Eigen::Ref<Eigen::ArrayXd> hut, Eigen::Ref<Eigen::ArrayXd> hul, double t)
{
    assert(rc.rows() == _roof_collision_dimension);
    assert(gc.rows() == _ground_collision_dimension);
    assert(sc.rows() == _self_collision_dimension);
    assert(soc.rows() == _static_obstacle_dimension);
    assert(doc.rows() == _dynamic_obstacle_dimension);
    assert(poc.rows() == _plane_collision_dimension);
    assert(hhc.rows() == _human_head_collision_dimension);
    assert(htc.rows() == _human_trunk_collision_dimension);
    assert(hlc.rows() == _human_limbs_collision_dimension);
    assert(huh.rows() == _human_uncertainty_head_collision_dimension);
    assert(hut.rows() == _human_uncertainty_trunk_collision_dimension);
    assert(hul.rows() == _human_uncertainty_limbs_collision_dimension);

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
        np = _plane_obstacle_list.size();
    }
    int nd = _dynamic_obstacle_list.size();
    int nh = _human_obstacle_list.size();

    if (_self_collision)
    {
        if (_use_preprocessor)
        {
            if (_min_distances)
            {
                sc = _preprocessor->_vector_data[1];
            }
            else
            {
                sc = _preprocessor->_vector_data[0];
            }
        }
        else
        {
            if (_min_distances)
            {
                sc(0) = _robot_collision->getMinSelfCollisionDistance();
            }
            else
            {
                _robot_collision->getSelfCollisionDistances(sc.head(_robot_collision->_self_collision_distance_dimension));
            }
        }
    }

    if (_ground_collision)
    {
        if (_use_preprocessor)
        {
            if (_min_distances)
            {
                gc = _preprocessor->_vector_data[5];
            }
            else
            {
                gc = _preprocessor->_vector_data[4];
            }
        }
        else
        {
            if (_min_distances)
            {
                gc(0) = _robot_collision->getMinGroundCollisionDistance();
            }
            else
            {
                _robot_collision->getGroundCollisionDistances(gc.head(_robot_collision->_plane_collision_distance_dimension));
            }
        }
    }

    if (_roof_collision)
    {
        if (_use_preprocessor)
        {
            if (_min_distances)
            {
                rc = _preprocessor->_vector_data[7];
            }
            else
            {
                rc = _preprocessor->_vector_data[6];
            }
        }
        else
        {
            if (_min_distances)
            {
                rc(0) = _robot_collision->getMinRoofCollisionDistance();
            }
            else
            {
                _robot_collision->getRoofCollisionDistances(rc.head(_robot_collision->_plane_collision_distance_dimension));
            }
        }
    }

    if (_static_obstacles)
    {
        if (_use_preprocessor)
        {
            if (_min_distances)
            {
                soc = _preprocessor->_vector_data[3];
            }
            else
            {
                soc = _preprocessor->_vector_data[2];
            }
        }
        else
        {
            for (int i = 0; i < ns; ++i)
            {
                if (_min_distances)
                {
                    soc(i) = _robot_collision->getMinObstacleDistance(_static_obstacle_list[i]);
                }
                else
                {
                    Eigen::Ref<Eigen::VectorXd> mask =
                        soc.segment(i * _robot_collision->_obstacle_distance_dimension, _robot_collision->_obstacle_distance_dimension);

                    _robot_collision->getObstacleDistances(_static_obstacle_list[i], mask);
                }
            }
        }
    }

    if (_dynamic_obstacles)
    {
        for (int i = 0; i < nd; ++i)
        {
            if (_use_preprocessor)
            {
                if (_min_distances)
                {
                    doc(i) = _preprocessor->_robot_collision->getMinObstacleDistance(_dynamic_obstacle_list[i], t);
                }
                else
                {
                    Eigen::Ref<Eigen::VectorXd> mask =
                        doc.segment(i * _robot_collision->_obstacle_distance_dimension, _robot_collision->_obstacle_distance_dimension);

                    _preprocessor->_robot_collision->getObstacleDistances(_dynamic_obstacle_list[i], mask, t);
                }
            }
            else
            {
                if (_min_distances)
                {
                    doc(i) = _robot_collision->getMinObstacleDistance(_dynamic_obstacle_list[i], t);
                }
                else
                {
                    Eigen::Ref<Eigen::VectorXd> mask =
                        doc.segment(i * _robot_collision->_obstacle_distance_dimension, _robot_collision->_obstacle_distance_dimension);

                    _robot_collision->getObstacleDistances(_dynamic_obstacle_list[i], mask, t);
                }
            }
        }
    }

    if (_human_head_collision || _human_trunk_collision || _human_limbs_collision)
    {
        for (int i = 0; i < nh; ++i)
        {
            if (_use_preprocessor)
            {
                if (_min_distances)
                {
                    if (_human_head_collision) hhc(i) = _preprocessor->_robot_collision->getMinHumanHeadDistance(_human_obstacle_list[i], t);

                    if (_human_trunk_collision)
                        _preprocessor->_robot_collision->getMinHumanTrunkDistance(_human_obstacle_list[i], htc.segment<2>(i * 2), t);

                    if (_human_limbs_collision)
                        _preprocessor->_robot_collision->getMinHumanLimbsDistance(_human_obstacle_list[i], hlc.segment<4>(i * 4), t);
                }
                else
                {
                    if (_human_head_collision)
                    {
                        Eigen::Ref<Eigen::VectorXd> mask_head = hhc.segment(i * _robot_collision->_human_collision_distance_dimension,
                                                                            _robot_collision->_human_collision_distance_dimension);
                        _preprocessor->_robot_collision->getHumanHeadDistances(_human_obstacle_list[i], mask_head, t);
                    }
                    if (_human_trunk_collision)
                    {
                        Eigen::Ref<Eigen::VectorXd> mask_trunk = htc.segment(i * 2 * _robot_collision->_human_collision_distance_dimension,
                                                                             2 * _robot_collision->_human_collision_distance_dimension);
                        _preprocessor->_robot_collision->getHumanTrunkDistances(_human_obstacle_list[i], mask_trunk, t);
                    }
                    if (_human_limbs_collision)
                    {
                        Eigen::Ref<Eigen::VectorXd> mask_limbs = hlc.segment(i * 4 * _robot_collision->_human_collision_distance_dimension,
                                                                             4 * _robot_collision->_human_collision_distance_dimension);
                        _preprocessor->_robot_collision->getHumanLimbsDistances(_human_obstacle_list[i], mask_limbs, t);
                    }
                }
            }
            else
            {
                if (_min_distances)
                {
                    if (_uncertainty_estimation_human)
                    {
                        if (_ms_planner_mode)
                        {
                            // First controller without Uncertainty stuff
                            if (_planner_id == 1)
                            {
                                if (_human_head_collision) hhc(i) = _robot_collision->getMinHumanHeadDistance(_human_obstacle_list[i], t);
                                if (_human_trunk_collision)
                                    _robot_collision->getMinHumanTrunkDistance(_human_obstacle_list[i], htc.segment<2>(i * 2), t);
                                if (_human_limbs_collision)
                                    _robot_collision->getMinHumanLimbsDistance(_human_obstacle_list[i], hlc.segment<4>(i * 4), t);
                            }
                            else
                            {
                                // Standard Distance Calculations for one uncertainty distance --> no consideration of other uncertainty variants
                                if (_human_head_collision)
                                    hhc(i) = _robot_collision->getMinHumanHeadDistanceUncertaintyInstance(
                                        _human_obstacle_list[i], _human_obstacle_list[i]._ms_mapping[_planner_id - 2], t);
                                if (_human_trunk_collision)
                                    _robot_collision->getMinHumanTrunkDistanceUncertaintyInstance(
                                        _human_obstacle_list[i], htc.segment<2>(i * 2), _human_obstacle_list[i]._ms_mapping[_planner_id - 2], t);
                                if (_human_limbs_collision)
                                {
                                    _robot_collision->getMinHumanLimbsDistanceUncertaintyInstance(
                                        _human_obstacle_list[i], hlc.segment<4>(i * 4), _human_obstacle_list[i]._ms_mapping[_planner_id - 2], t);
                                }
                            }
                        }
                        else
                        {
                            // Standard Distance Calculations
                            if (_human_head_collision) hhc(i) = _robot_collision->getMinHumanHeadDistance(_human_obstacle_list[i], t);
                            if (_human_trunk_collision) _robot_collision->getMinHumanTrunkDistance(_human_obstacle_list[i], htc.segment<2>(i * 2), t);
                            if (_human_limbs_collision) _robot_collision->getMinHumanLimbsDistance(_human_obstacle_list[i], hlc.segment<4>(i * 4), t);

                            // Radius Increase Distance calculations for uncertainty mode
                            if (_human_head_collision && _uncertainty_mode == RadiusIncrease)
                                huh(i) = _robot_collision->getMinHumanHeadDistanceRadiusIncrease(_human_obstacle_list[i], t, t);
                            if (_human_trunk_collision && _uncertainty_mode == RadiusIncrease)
                            {
                                Eigen::Ref<Eigen::VectorXd> mask_uncertainty_trunk =
                                    hut.segment(i * _human_uncertainty_trunk_collision_dimension, _human_uncertainty_trunk_collision_dimension);
                                _robot_collision->getMinHumanTrunkDistanceRadiusIncrease(_human_obstacle_list[i], mask_uncertainty_trunk, t, t);
                            }
                            if (_human_limbs_collision && _uncertainty_mode == RadiusIncrease)
                            {
                                Eigen::Ref<Eigen::VectorXd> mask_uncertainty_limbs =
                                    hul.segment(i * _human_uncertainty_limbs_collision_dimension, _human_uncertainty_limbs_collision_dimension);
                                _robot_collision->getMinHumanLimbsDistanceRadiusIncrease(_human_obstacle_list[i], mask_uncertainty_limbs, t, t);
                            }

                            // Skeleton splitting for uncertainy mode
                            if (_human_head_collision && _human_uncertainty_head_collision_dimension > 0 && _uncertainty_mode == SkeletonSplitting)
                            {
                                huh(i) = _robot_collision->getMinHumanUncertaintyHeadDistance(_human_obstacle_list[i], t);
                            }
                            if (_human_trunk_collision && _human_uncertainty_trunk_collision_dimension > 0 && _uncertainty_mode == SkeletonSplitting)
                            {
                                Eigen::Ref<Eigen::VectorXd> mask_uncertainty_trunk =
                                    hut.segment(i * _human_uncertainty_trunk_collision_dimension, _human_uncertainty_trunk_collision_dimension);
                                _robot_collision->getMinHumanUncertaintyTrunkDistance(_human_obstacle_list[i], mask_uncertainty_trunk, t);
                            }
                            if (_human_limbs_collision && _human_uncertainty_limbs_collision_dimension > 0 && _uncertainty_mode == SkeletonSplitting)
                            {
                                Eigen::Ref<Eigen::VectorXd> mask_uncertainty_limbs =
                                    hul.segment(i * _human_uncertainty_limbs_collision_dimension, _human_uncertainty_limbs_collision_dimension);
                                _robot_collision->getMinHumanUncertaintyLimbsDistance(_human_obstacle_list[i], mask_uncertainty_limbs, t);
                            }
                        }
                    }
                    else
                    {
                        if (_human_head_collision) hhc(i) = _robot_collision->getMinHumanHeadDistance(_human_obstacle_list[i], t);
                        if (_human_trunk_collision) _robot_collision->getMinHumanTrunkDistance(_human_obstacle_list[i], htc.segment<2>(i * 2), t);
                        if (_human_limbs_collision) _robot_collision->getMinHumanLimbsDistance(_human_obstacle_list[i], hlc.segment<4>(i * 4), t);
                    }
                }
                else
                {
                    if (_uncertainty_estimation_human)
                    {
                        if (_ms_planner_mode)
                        {
                            if (_planner_id == 1)
                            {
                                // Planner 1 uses the default distance between human and robot
                                if (_human_head_collision)
                                {
                                    Eigen::Ref<Eigen::VectorXd> mask_head = hhc.segment(i * _robot_collision->_human_collision_distance_dimension,
                                                                                        _robot_collision->_human_collision_distance_dimension);
                                    _robot_collision->getHumanHeadDistances(_human_obstacle_list[i], mask_head, t);
                                }
                                if (_human_trunk_collision)
                                {
                                    Eigen::Ref<Eigen::VectorXd> mask_trunk =
                                        htc.segment(i * 2 * _robot_collision->_human_collision_distance_dimension,
                                                    2 * _robot_collision->_human_collision_distance_dimension);
                                    _robot_collision->getHumanTrunkDistances(_human_obstacle_list[i], mask_trunk, t);
                                }
                                if (_human_limbs_collision)
                                {
                                    Eigen::Ref<Eigen::VectorXd> mask_limbs =
                                        hlc.segment(i * 4 * _robot_collision->_human_collision_distance_dimension,
                                                    4 * _robot_collision->_human_collision_distance_dimension);
                                    _robot_collision->getHumanLimbsDistances(_human_obstacle_list[i], mask_limbs, t);
                                }
                            }
                            else
                            {  // Standard Distance Calculations for one uncertainty variant --> no consideration of other uncertainty variants
                                if (_human_head_collision)
                                {
                                    Eigen::Ref<Eigen::VectorXd> mask_head = hhc.segment(i * _robot_collision->_human_collision_distance_dimension,
                                                                                        _robot_collision->_human_collision_distance_dimension);
                                    _robot_collision->getHumanHeadDistancesUncertaintyInstance(
                                        _human_obstacle_list[i], mask_head, _human_obstacle_list[i]._ms_mapping[_planner_id - 2], t);
                                }
                                if (_human_trunk_collision)
                                {
                                    Eigen::Ref<Eigen::VectorXd> mask_trunk =
                                        htc.segment(i * 2 * _robot_collision->_human_collision_distance_dimension,
                                                    2 * _robot_collision->_human_collision_distance_dimension);
                                    _robot_collision->getHumanTrunkDistancesUncertaintyInstance(
                                        _human_obstacle_list[i], mask_trunk, _human_obstacle_list[i]._ms_mapping[_planner_id - 2], t);
                                }
                                if (_human_limbs_collision)
                                {
                                    Eigen::Ref<Eigen::VectorXd> mask_limbs =
                                        hlc.segment(i * 4 * _robot_collision->_human_collision_distance_dimension,
                                                    4 * _robot_collision->_human_collision_distance_dimension);
                                    _robot_collision->getHumanLimbsDistancesUncertaintyInstance(
                                        _human_obstacle_list[i], mask_limbs, _human_obstacle_list[i]._ms_mapping[_planner_id - 2], t);
                                }
                            }
                        }
                        else
                        {
                            // No uncertainty estimation
                            if (_human_head_collision)
                            {
                                Eigen::Ref<Eigen::VectorXd> mask_head = hhc.segment(i * _robot_collision->_human_collision_distance_dimension,
                                                                                    _robot_collision->_human_collision_distance_dimension);
                                _robot_collision->getHumanHeadDistances(_human_obstacle_list[i], mask_head, t);
                            }
                            if (_human_trunk_collision)
                            {
                                Eigen::Ref<Eigen::VectorXd> mask_trunk = htc.segment(i * 2 * _robot_collision->_human_collision_distance_dimension,
                                                                                     2 * _robot_collision->_human_collision_distance_dimension);
                                _robot_collision->getHumanTrunkDistances(_human_obstacle_list[i], mask_trunk, t);
                            }
                            if (_human_limbs_collision)
                            {
                                Eigen::Ref<Eigen::VectorXd> mask_limbs = hlc.segment(i * 4 * _robot_collision->_human_collision_distance_dimension,
                                                                                     4 * _robot_collision->_human_collision_distance_dimension);
                                _robot_collision->getHumanLimbsDistances(_human_obstacle_list[i], mask_limbs, t);
                            }

                            // Radius Increase
                            if (_human_head_collision && _uncertainty_mode == RadiusIncrease)
                            {
                                Eigen::Ref<Eigen::VectorXd> mask_head = huh.segment(i * _robot_collision->_human_collision_distance_dimension,
                                                                                    _robot_collision->_human_collision_distance_dimension);

                                _robot_collision->getHumanHeadDistancesRadiusIncrease(_human_obstacle_list[i], mask_head, t, t);
                            }
                            if (_human_trunk_collision && _uncertainty_mode == RadiusIncrease)
                            {
                                Eigen::Ref<Eigen::VectorXd> mask_trunk = hut.segment(i * 2 * _robot_collision->_human_collision_distance_dimension,
                                                                                     2 * _robot_collision->_human_collision_distance_dimension);
                                _robot_collision->getHumanTrunkDistancesRadiusIncrease(_human_obstacle_list[i], mask_trunk, t, t);
                            }
                            if (_human_limbs_collision && _uncertainty_mode == RadiusIncrease)
                            {
                                Eigen::Ref<Eigen::VectorXd> mask_limbs = hul.segment(i * 4 * _robot_collision->_human_collision_distance_dimension,
                                                                                     4 * _robot_collision->_human_collision_distance_dimension);
                                _robot_collision->getHumanLimbsDistancesRadiusIncrease(_human_obstacle_list[i], mask_limbs, t, t);
                            }

                            // Skeleton Splitting
                            if (_human_head_collision && _human_uncertainty_head_collision_dimension > 0 && _uncertainty_mode == SkeletonSplitting)
                            {
                                Eigen::Ref<Eigen::VectorXd> mask_uncertainty_head =
                                    huh.segment(i * _human_uncertainty_head_collision_dimension, _human_uncertainty_head_collision_dimension);
                                _robot_collision->getHumanUncertaintyHeadDistances(_human_obstacle_list[i], mask_uncertainty_head, t);
                            }
                            if (_human_trunk_collision && _human_uncertainty_trunk_collision_dimension > 0 && _uncertainty_mode == SkeletonSplitting)
                            {
                                Eigen::Ref<Eigen::VectorXd> mask_uncertainty_trunk =
                                    hut.segment(i * _human_uncertainty_trunk_collision_dimension, _human_uncertainty_trunk_collision_dimension);
                                _robot_collision->getHumanUncertaintyTrunkDistances(_human_obstacle_list[i], mask_uncertainty_trunk, t);
                            }
                            if (_human_limbs_collision && _human_uncertainty_limbs_collision_dimension > 0 && _uncertainty_mode == SkeletonSplitting)
                            {
                                Eigen::Ref<Eigen::VectorXd> mask_uncertainty_limbs =
                                    hul.segment(i * _human_uncertainty_limbs_collision_dimension, _human_uncertainty_limbs_collision_dimension);
                                _robot_collision->getHumanUncertaintyLimbsDistances(_human_obstacle_list[i], mask_uncertainty_limbs, t);
                            }
                        }
                    }
                    else
                    {
                        if (_human_head_collision)
                        {
                            Eigen::Ref<Eigen::VectorXd> mask_head = hhc.segment(i * _robot_collision->_human_collision_distance_dimension,
                                                                                _robot_collision->_human_collision_distance_dimension);
                            _robot_collision->getHumanHeadDistances(_human_obstacle_list[i], mask_head, t);
                        }
                        if (_human_trunk_collision)
                        {
                            Eigen::Ref<Eigen::VectorXd> mask_trunk = htc.segment(i * 2 * _robot_collision->_human_collision_distance_dimension,
                                                                                 2 * _robot_collision->_human_collision_distance_dimension);
                            _robot_collision->getHumanTrunkDistances(_human_obstacle_list[i], mask_trunk, t);
                        }
                        if (_human_limbs_collision)
                        {
                            Eigen::Ref<Eigen::VectorXd> mask_limbs = hlc.segment(i * 4 * _robot_collision->_human_collision_distance_dimension,
                                                                                 4 * _robot_collision->_human_collision_distance_dimension);
                            _robot_collision->getHumanLimbsDistances(_human_obstacle_list[i], mask_limbs, t);
                        }
                    }
                }
            }
        }
    }

    if (_plane_collision)
    {
        if (_use_preprocessor)
        {
            if (_min_distances)
            {
                poc = _preprocessor->_vector_data[9];
            }
            else
            {
                poc = _preprocessor->_vector_data[8];
            }
        }
        else
        {
            for (int i = 0; i < np; ++i)
            {
                if (_min_distances)
                {
                    poc(i) = _robot_collision->getMinPlaneCollisionDistance(_plane_obstacle_list[i]);
                }
                else
                {
                    Eigen::Ref<Eigen::VectorXd> mask =
                        poc.segment(i * _robot_collision->_plane_collision_distance_dimension, _robot_collision->_plane_collision_distance_dimension);

                    _robot_collision->getPlaneCollisionDistances(_plane_obstacle_list[i], mask);
                }
            }
        }
    }

    assert(rc.rows() == _roof_collision_dimension);
    assert(gc.rows() == _ground_collision_dimension);
    assert(sc.rows() == _self_collision_dimension);
    assert(soc.rows() == _static_obstacle_dimension);
    assert(doc.rows() == _dynamic_obstacle_dimension);
    assert(poc.rows() == _plane_collision_dimension);
    assert(hhc.rows() == _human_head_collision_dimension);
    assert(htc.rows() == _human_trunk_collision_dimension);
    assert(hlc.rows() == _human_limbs_collision_dimension);
    assert(huh.rows() == _human_uncertainty_head_collision_dimension);
    assert(hut.rows() == _human_uncertainty_trunk_collision_dimension);
    assert(hul.rows() == _human_uncertainty_limbs_collision_dimension);
}

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot
