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

#include <mhp_robot/robot_trajectory_optimization/robot_collision_potential_mohri.h>

namespace mhp_robot {
namespace robot_trajectory_optimization {

double RobotCollisionPotentialMohri::computePotentials(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k)
{
    Eigen::ArrayXd sc_distances(_self_collision_dimension);
    Eigen::ArrayXd so_distances(_static_obstacle_dimension);
    Eigen::ArrayXd do_distances(_dynamic_obstacle_dimension);
    Eigen::ArrayXd gc_distances(_ground_collision_dimension);
    Eigen::ArrayXd rc_distances(_roof_collision_dimension);
    Eigen::ArrayXd pc_distances(_plane_collision_dimension);
    Eigen::ArrayXd hh_distances(_human_head_collision_dimension);
    Eigen::ArrayXd ht_distances(_human_trunk_collision_dimension);
    Eigen::ArrayXd hl_distances(_human_limbs_collision_dimension);
    Eigen::ArrayXd huh_distances(_human_uncertainty_head_collision_dimension);
    Eigen::ArrayXd hut_distances(_human_uncertainty_trunk_collision_dimension);
    Eigen::ArrayXd hul_distances(_human_uncertainty_limbs_collision_dimension);

    double t = (_obstacle_prediction ? k * _dt : 0.0);
    _planning_delay._mutex.lock();
    t += (_prediction_offest ? _planning_delay._delay : 0.0);
    _planning_delay._mutex.unlock();
    double potential = 0;

    computeDistances(x_k, rc_distances, gc_distances, sc_distances, so_distances, do_distances, pc_distances, hh_distances, ht_distances,
                     hl_distances, huh_distances, hut_distances, hul_distances, t);

    if (_self_collision)
    {
        sc_distances = sc_distances.cwiseMin(_margins(0));

        if (_extended)
        {
            potential += _weights(0) * (((sc_distances / _margins(0)) - 1).square()).sum();
        }
        else
        {
            potential += _weights(0) * ((sc_distances - _margins(0)).square()).sum();
        }
    }

    if (_static_obstacles)
    {
        so_distances = so_distances.cwiseMin(_margins(1));

        if (_extended)
        {
            potential += _weights(1) * (((so_distances / _margins(1)) - 1).square()).sum();
        }
        else
        {
            potential += _weights(1) * ((so_distances - _margins(1)).square()).sum();
        }
    }

    if (_dynamic_obstacles)
    {
        do_distances = do_distances.cwiseMin(_margins(2));

        if (_extended)
        {
            potential += _weights(2) * (((do_distances / _margins(2)) - 1).square()).sum();
        }
        else
        {
            potential += _weights(2) * ((do_distances - _margins(2)).square()).sum();
        }
    }

    if (_ground_collision)
    {
        gc_distances = gc_distances.cwiseMin(_margins(3));

        if (_extended)
        {
            potential += _weights(3) * (((gc_distances / _margins(3)) - 1).square()).sum();
        }
        else
        {
            potential += _weights(3) * ((gc_distances - _margins(3)).square()).sum();
        }
    }

    if (_roof_collision)
    {
        rc_distances = rc_distances.cwiseMin(_margins(4));

        if (_extended)
        {
            potential += _weights(4) * (((rc_distances / _margins(4)) - 1).square()).sum();
        }
        else
        {
            potential += _weights(4) * ((rc_distances - _margins(4)).square()).sum();
        }
    }

    if (_plane_collision)
    {
        pc_distances = pc_distances.cwiseMin(_margins(5));

        if (_extended)
        {
            potential += _weights(5) * (((pc_distances / _margins(5)) - 1).square()).sum();
        }
        else
        {
            potential += _weights(5) * ((pc_distances - _margins(5)).square()).sum();
        }
    }

    if (_human_head_collision || _human_trunk_collision || _human_limbs_collision)
    {
        if (_human_head_collision) hh_distances = hh_distances.cwiseMin(_margins(6));
        if (_human_trunk_collision) ht_distances = ht_distances.cwiseMin(_margins(7));
        //        std::cout << "HL Distnaces Size: " << hl_distances.rows() << " " << hl_distances.cols() << std::endl;
        if (_human_limbs_collision) hl_distances = hl_distances.cwiseMin(_margins(8));
        //        std::cout << "HL Distnaces Size: " << hl_distances.rows() << " " << hl_distances.cols() << std::endl;

        if (_extended)
        {
            if (_human_head_collision) potential += _weights(6) * (((hh_distances / _margins(6)) - 1).square()).sum();
            if (_human_trunk_collision) potential += _weights(7) * (((ht_distances / _margins(7)) - 1).square()).sum();
            if (_human_limbs_collision) potential += _weights(8) * (((hl_distances / _margins(8)) - 1).square()).sum();
        }
        else
        {
            if (_human_head_collision) potential += _weights(6) * ((hh_distances - _margins(6)).square()).sum();
            if (_human_trunk_collision) potential += _weights(7) * ((ht_distances - _margins(7)).square()).sum();
            if (_human_limbs_collision) potential += _weights(8) * ((hl_distances - _margins(8)).square()).sum();
        }
        if (_uncertainty_estimation_human)
        {
            if (_human_head_collision) huh_distances = huh_distances.cwiseMin(_margins(6));
            if (_human_trunk_collision) hut_distances = hut_distances.cwiseMin(_margins(7));
            if (_human_limbs_collision) hul_distances = hul_distances.cwiseMin(_margins(8));
            if (_extended)
            {
                if (_human_head_collision) potential += _weights(6) * (((huh_distances / _margins(6)) - 1).square()).sum();
                if (_human_trunk_collision) potential += _weights(7) * (((hut_distances / _margins(7)) - 1).square()).sum();
                if (_human_limbs_collision) potential += _weights(8) * (((hul_distances / _margins(8)) - 1).square()).sum();
            }
            else
            {
                if (_human_head_collision) potential += _weights(6) * ((huh_distances - _margins(6)).square()).sum();
                if (_human_trunk_collision) potential += _weights(7) * ((hut_distances - _margins(7)).square()).sum();
                if (_human_limbs_collision) potential += _weights(8) * ((hul_distances - _margins(8)).square()).sum();
            }
        }
    }

    return potential;
}  // namespace robot_trajectory_optimization

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot
