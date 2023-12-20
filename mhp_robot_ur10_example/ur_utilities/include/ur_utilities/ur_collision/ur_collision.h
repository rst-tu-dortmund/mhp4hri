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

#ifndef UR_COLLISION_H
#define UR_COLLISION_H

#include <mhp_robot/robot_collision/robot_collision.h>
#include <ur_utilities/ur_kinematic/ur_kinematic.h>

namespace mhp_robot {
namespace robot_collision {

class URCollision : public RobotCollision
{
 public:
    using Ptr  = std::shared_ptr<URCollision>;
    using UPtr = std::unique_ptr<URCollision>;

    URCollision();

    RobotCollision::UPtr createUniqueInstance() const override;
    RobotCollision::Ptr createSharedInstance() const override;

    // Robot - Robot
    bool getSelfCollisionDistances(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d) const override;
    bool getSelfCollisionDistances(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                   Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points) const override;

    // Robot - Obstacle
    bool getObstacleDistances(const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                              double t_radius = 0) const override;
    bool getObstacleDistances(const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                              Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                              double t = 0, double t_radius = 0) const override;

    // Robot - Uncertainty Obstacle
    bool getUncertaintyObstacleDistances(const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                         int iterator = 0) const override;
    bool getUncertaintyObstacleDistances(const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                         Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                         double t = 0, int iterator = 0) const override;

    // Robot - Plane
    bool getPlaneCollisionDistances(const robot_misc::Plane& plane, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d) const override;
    bool getPlaneCollisionDistances(const robot_misc::Plane& plane, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                    Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                    Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points) const override;
};

}  // namespace robot_collision
}  // namespace mhp_robot

#endif  // UR_COLLISION_H
