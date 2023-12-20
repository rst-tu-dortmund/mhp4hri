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
#include <ros/io.h>
#include <ur_utilities/ur_collision/ur_collision.h>

namespace mhp_robot {
namespace robot_collision {

URCollision::URCollision() : RobotCollision(std::make_shared<robot_kinematic::URKinematic>(), 6, 5, 8) {}

RobotCollision::UPtr URCollision::createUniqueInstance() const { return std::make_unique<URCollision>(); }

RobotCollision::Ptr URCollision::createSharedInstance() const { return std::make_shared<URCollision>(); }

bool URCollision::getSelfCollisionDistances(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d) const
{
    bool success = true;

    // using +1 on all links starting with 1, because 1 is inactive
    success &= getDistanceBetweenLinks(d(0), 4, 0);
    success &= getDistanceBetweenLinks(d(1), 4, 2);
    success &= getDistanceBetweenLinks(d(2), 5, 3);
    success &= getDistanceBetweenLinks(d(3), 6, 3);
    success &= getDistanceBetweenLinks(d(4), 7, 0);
    success &= getDistanceBetweenLinks(d(5), 7, 2);
    success &= getDistanceBetweenLinks(d(6), 7, 3);
    success &= getDistanceBetweenLinks(d(7), 7, 4);

    if (!success)
    {
        ROS_WARN("URCollision: Error calculating self collision distances.");
    }

    return success;
}

bool URCollision::getSelfCollisionDistances(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                            Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points) const
{
    bool success = true;

    // using +1 on all links starting with 1, because 1 is inactive
    success &= getDistanceBetweenLinks(d(0), 4, 0, start_points.col(0), end_points.col(0));
    success &= getDistanceBetweenLinks(d(1), 4, 2, start_points.col(1), end_points.col(1));
    success &= getDistanceBetweenLinks(d(2), 5, 3, start_points.col(2), end_points.col(2));
    success &= getDistanceBetweenLinks(d(3), 6, 3, start_points.col(3), end_points.col(3));
    success &= getDistanceBetweenLinks(d(4), 7, 0, start_points.col(4), end_points.col(4));
    success &= getDistanceBetweenLinks(d(5), 7, 2, start_points.col(5), end_points.col(5));
    success &= getDistanceBetweenLinks(d(6), 7, 3, start_points.col(6), end_points.col(6));
    success &= getDistanceBetweenLinks(d(7), 7, 4, start_points.col(7), end_points.col(7));

    if (!success)
    {
        ROS_WARN("URCollision: Error calculating self collision distances.");
    }

    return success;
}

bool URCollision::getObstacleDistances(const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                       double t_radius) const
{
    bool success = true;

    // skip 0 because it cannot move anyway and 1 because it is inactive
    success &= getDistanceToObstacle(d(0), 2, obstacle, t, t_radius);
    success &= getDistanceToObstacle(d(1), 3, obstacle, t, t_radius);
    success &= getDistanceToObstacle(d(2), 4, obstacle, t, t_radius);
    success &= getDistanceToObstacle(d(3), 5, obstacle, t, t_radius);
    success &= getDistanceToObstacle(d(4), 6, obstacle, t, t_radius);
    success &= getDistanceToObstacle(d(5), 7, obstacle, t, t_radius);

    if (!success)
    {
        ROS_WARN("URCollision: Error calculating distances to obstacle.");
    }

    return success;
}

bool URCollision::getObstacleDistances(const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                       Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                       double t, double t_radius) const
{
    bool success = true;

    // skip 0 because it cannot move anyway and 1 because it is inactive
    success &= getDistanceToObstacle(d(0), 2, obstacle, start_points.col(0), end_points.col(0), t, t_radius);
    success &= getDistanceToObstacle(d(1), 3, obstacle, start_points.col(1), end_points.col(1), t, t_radius);
    success &= getDistanceToObstacle(d(2), 4, obstacle, start_points.col(2), end_points.col(2), t, t_radius);
    success &= getDistanceToObstacle(d(3), 5, obstacle, start_points.col(3), end_points.col(3), t, t_radius);
    success &= getDistanceToObstacle(d(4), 6, obstacle, start_points.col(4), end_points.col(4), t, t_radius);
    success &= getDistanceToObstacle(d(5), 7, obstacle, start_points.col(5), end_points.col(5), t, t_radius);
    if (!success)
    {
        ROS_WARN("URCollision: Error calculating distances to obstacle.");
    }

    return success;
}

bool URCollision::getUncertaintyObstacleDistances(const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                  Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                                  Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t, int iterator) const
{
    bool success = true;

    // skip 0 because it cannot move anyway and 1 because it is inactive
    success &= getDistanceToUncertaintyObstacle(d(0), 2, obstacle, start_points.col(0), end_points.col(0), t, iterator);
    success &= getDistanceToUncertaintyObstacle(d(1), 3, obstacle, start_points.col(1), end_points.col(1), t, iterator);
    success &= getDistanceToUncertaintyObstacle(d(2), 4, obstacle, start_points.col(2), end_points.col(2), t, iterator);
    success &= getDistanceToUncertaintyObstacle(d(3), 5, obstacle, start_points.col(3), end_points.col(3), t, iterator);
    success &= getDistanceToUncertaintyObstacle(d(4), 6, obstacle, start_points.col(4), end_points.col(4), t, iterator);
    success &= getDistanceToUncertaintyObstacle(d(5), 7, obstacle, start_points.col(5), end_points.col(5), t, iterator);

    if (!success || (d.array() == 0).any())
    {
        ROS_WARN("URCollision: Error calculating distances to uncertainty obstacle.");
    }

    return success;
}

bool URCollision::getUncertaintyObstacleDistances(const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                                  int iterator) const
{
    bool success = true;

    // skip 0 because it cannot move anyway and 1 because it is inactive
    success &= getDistanceToUncertaintyObstacle(d(0), 2, obstacle, t, iterator);
    success &= getDistanceToUncertaintyObstacle(d(1), 3, obstacle, t, iterator);
    success &= getDistanceToUncertaintyObstacle(d(2), 4, obstacle, t, iterator);
    success &= getDistanceToUncertaintyObstacle(d(3), 5, obstacle, t, iterator);
    success &= getDistanceToUncertaintyObstacle(d(4), 6, obstacle, t, iterator);
    success &= getDistanceToUncertaintyObstacle(d(5), 7, obstacle, t, iterator);
    if (!success || (d.array() == 0).any())
    {
        ROS_WARN("URCollision: Error calculating distances to uncertainty obstacle.");
    }

    return success;
}

bool URCollision::getPlaneCollisionDistances(const robot_misc::Plane& plane, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                             Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                             Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points) const
{
    bool success = true;

    // skip 0 because it can not move anyway, 1 because it is inactive and 2 because it cannot collide with a plane
    success &= getDistanceToPlane(d(0), 3, plane, start_points.col(0), end_points.col(0));
    success &= getDistanceToPlane(d(1), 4, plane, start_points.col(1), end_points.col(1));
    success &= getDistanceToPlane(d(2), 5, plane, start_points.col(2), end_points.col(2));
    success &= getDistanceToPlane(d(3), 6, plane, start_points.col(3), end_points.col(3));
    success &= getDistanceToPlane(d(4), 7, plane, start_points.col(4), end_points.col(4));

    if (!success)
    {
        ROS_WARN("URCollision: Error calculating distances to plane.");
    }

    return success;
}

bool URCollision::getPlaneCollisionDistances(const robot_misc::Plane& plane, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d) const
{
    bool success = true;

    // skip 0 because it can not move anyway, 1 because it is inactive and 2 because it cannot collide with a plane
    success &= getDistanceToPlane(d(0), 3, plane);
    success &= getDistanceToPlane(d(1), 4, plane);
    success &= getDistanceToPlane(d(2), 5, plane);
    success &= getDistanceToPlane(d(3), 6, plane);
    success &= getDistanceToPlane(d(4), 7, plane);

    if (!success)
    {
        ROS_WARN("URCollision: Error calculating distances to plane.");
    }

    return success;
}

}  // namespace robot_collision
}  // namespace mhp_robot
