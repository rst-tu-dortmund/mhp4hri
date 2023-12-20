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

#include <mhp_robot/robot_collision/robot_collision.h>
#include <chrono>
#include <random>

namespace mhp_robot {
namespace robot_collision {

RobotCollision::RobotCollision(robot_kinematic::RobotKinematic::Ptr robot_kinematic, int obstacle_dim, int plane_dim, int self_collision_dim)
    : _obstacle_distance_dimension(obstacle_dim),
      _plane_collision_distance_dimension(plane_dim),
      _self_collision_distance_dimension(self_collision_dim),
      _human_collision_distance_dimension(obstacle_dim),
      _robot_kinematic(robot_kinematic)
{
    _segment_structs = _robot_kinematic->getRobotUtility().getRobotDescription().getKinematicStructure();
    _ground_plane    = _robot_kinematic->getRobotUtility().getRobotDescription().getGroundPlane();
    _roof_plane      = _robot_kinematic->getRobotUtility().getRobotDescription().getRoofPlane();
}

robot_kinematic::RobotKinematic::Ptr RobotCollision::getRobotKinematic() { return _robot_kinematic; }

bool RobotCollision::getDistanceBetweenLinks(double& d, int j, int k) const
{
    Eigen::Matrix<double, 3, 1> start_point, end_point;
    return getDistanceBetweenLinks(d, j, k, start_point, end_point);
}

bool RobotCollision::getDistanceBetweenLinks(double& d, int j, int k, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                             Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point) const
{
    const robot_misc::Segment& seg1 = _segment_structs[j];
    const robot_misc::Segment& seg2 = _segment_structs[k];

    // check if at least one segment bbox is inactive
    if (!seg1.bounding_box.active || !seg2.bounding_box.active) return false;

    const std::vector<Eigen::Matrix<double, 4, 4>>& full_direct_kinematics = _robot_kinematic->getForwardKinematicChain();

    robot_misc::BoundingBoxType t1, t2;
    t1 = seg1.bounding_box.type;
    t2 = seg2.bounding_box.type;

    Eigen::Matrix<double, 4, 1> end;
    end << 0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4d T1, T2;

    if (t1 == robot_misc::BoundingBoxType::CYLINDER && t2 == robot_misc::BoundingBoxType::CYLINDER)
    {
        // cylinder - cylinder
        Eigen::Matrix<double, 4, 1> p1, p2, q1, q2;

        T1.noalias() = full_direct_kinematics[j] * seg1.bounding_box.T;
        T2.noalias() = full_direct_kinematics[k] * seg2.bounding_box.T;

        p1 = T1.block<4, 1>(0, 3);

        end(0)       = seg1.bounding_box.length_x;
        p2.noalias() = T1 * end;
        q1           = T2.block<4, 1>(0, 3);

        end(0)       = seg2.bounding_box.length_x;
        q2.noalias() = T2 * end;

        d = getLineToLineDistance(p1.head<3>(), p2.head<3>(), q1.head<3>(), q2.head<3>(), start_point, end_point) - seg1.bounding_box.radius -
            seg2.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::CYLINDER && t2 == robot_misc::BoundingBoxType::SPHERE)
    {
        // cylinder - spehre
        Eigen::Matrix<double, 4, 1> p1, p2, q;
        T1.noalias() = full_direct_kinematics[j] * seg1.bounding_box.T;

        p1 = T1.block<4, 1>(0, 3);

        end(0)       = seg1.bounding_box.length_x;
        p2.noalias() = T1 * end;
        q.noalias()  = (full_direct_kinematics[k] * seg2.bounding_box.T).block<4, 1>(0, 3);

        d = getPointToLineDistance(q.head<3>(), p1.head<3>(), p2.head<3>(), end_point, start_point) - seg1.bounding_box.radius -
            seg2.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE && t2 == robot_misc::BoundingBoxType::CYLINDER)
    {
        // sphere - cylinder
        Eigen::Matrix<double, 4, 1> p, q1, q2;
        T2.noalias() = full_direct_kinematics[k] * seg2.bounding_box.T;

        p.noalias() = (full_direct_kinematics[j] * seg1.bounding_box.T).block<4, 1>(0, 3);
        q1          = T2.block<4, 1>(0, 3);

        end(0)       = seg2.bounding_box.length_x;
        q2.noalias() = T2 * end;

        d = getPointToLineDistance(p.head<3>(), q1.head<3>(), q2.head<3>(), start_point, end_point) - seg1.bounding_box.radius -
            seg2.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE && t2 == robot_misc::BoundingBoxType::SPHERE)
    {
        // sphere - sphere
        Eigen::Matrix<double, 4, 1> p1, p2;

        p1.noalias() = (full_direct_kinematics[j] * seg1.bounding_box.T).block<4, 1>(0, 3);
        p2.noalias() = (full_direct_kinematics[k] * seg2.bounding_box.T).block<4, 1>(0, 3);

        d = getPointToPointDistance(p1.head<3>(), p2.head<3>(), start_point, end_point) - seg1.bounding_box.radius - seg2.bounding_box.radius;

        return true;
    }
    return false;
}

bool RobotCollision::getDistanceToObstacle(double& d, int j, const robot_misc::Obstacle& obstacle, double t, double t_radius) const
{
    Eigen::Matrix<double, 3, 1> start_point, end_point;
    return getDistanceToObstacle(d, j, obstacle, start_point, end_point, t, t_radius);
}

bool RobotCollision::getDistanceToObstacle(double& d, int j, const robot_misc::Obstacle& obstacle,
                                           Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point, Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point,
                                           double t, double t_radius) const
{
    const robot_misc::Segment& seg = _segment_structs[j];

    // check if segment bbox is inactive
    if (!seg.bounding_box.active) return false;

    const std::vector<Eigen::Matrix<double, 4, 4>>& full_direct_kinematics = _robot_kinematic->getForwardKinematicChain();

    robot_misc::BoundingBoxType t1, t2;
    t1 = seg.bounding_box.type;
    t2 = obstacle.bounding_box.type;

    Eigen::Matrix<double, 4, 1> end;
    end << 0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4d T1, T2;

    if (t1 == robot_misc::BoundingBoxType::CYLINDER && t2 == robot_misc::BoundingBoxType::CYLINDER)
    {
        // cylinder - cylinder
        Eigen::Matrix<double, 4, 1> p1, p2, q1, q2;
        T1.noalias() = full_direct_kinematics[j] * seg.bounding_box.T;
        T2.noalias() = obstacle.state.getPose(t) * obstacle.bounding_box.T;

        p1 = T1.block<4, 1>(0, 3);

        end(0)       = seg.bounding_box.length_x;
        p2.noalias() = T1 * end;
        q1           = T2.block<4, 1>(0, 3);

        end(0)       = obstacle.bounding_box.length_x;
        q2.noalias() = T2 * end;

        d = getLineToLineDistance(p1.head<3>(), p2.head<3>(), q1.head<3>(), q2.head<3>(), start_point, end_point) - seg.bounding_box.getRadius() -
            obstacle.bounding_box.getRadius(t_radius);

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::CYLINDER && t2 == robot_misc::BoundingBoxType::SPHERE)
    {
        // cylinder - spehre
        Eigen::Matrix<double, 4, 1> p1, p2, q;
        T1.noalias() = full_direct_kinematics[j] * seg.bounding_box.T;

        p1 = T1.block<4, 1>(0, 3);

        end(0)       = seg.bounding_box.length_x;
        p2.noalias() = T1 * end;
        q.noalias()  = (obstacle.state.getPose(t) * obstacle.bounding_box.T).block<4, 1>(0, 3);

        d = getPointToLineDistance(q.head<3>(), p1.head<3>(), p2.head<3>(), end_point, start_point) - seg.bounding_box.radius -
            obstacle.bounding_box.getRadius(t_radius);

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE && t2 == robot_misc::BoundingBoxType::CYLINDER)
    {
        // sphere - cylinder
        Eigen::Matrix<double, 4, 1> p, q1, q2;
        T2.noalias() = obstacle.state.getPose(t) * obstacle.bounding_box.T;

        p  = (full_direct_kinematics[j] * seg.bounding_box.T).block<4, 1>(0, 3);
        q1 = T2.block<4, 1>(0, 3);

        end(0)       = obstacle.bounding_box.length_x;
        q2.noalias() = T2 * end;

        d = getPointToLineDistance(p.head<3>(), q1.head<3>(), q2.head<3>(), start_point, end_point) - seg.bounding_box.radius -
            obstacle.bounding_box.getRadius(t_radius);

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE && t2 == robot_misc::BoundingBoxType::SPHERE)
    {
        // sphere - sphere
        Eigen::Matrix<double, 4, 1> p1, p2;

        p1.noalias() = (full_direct_kinematics[j] * seg.bounding_box.T).block<4, 1>(0, 3);
        p2.noalias() = (obstacle.state.getPose(t) * obstacle.bounding_box.T).block<4, 1>(0, 3);

        d = getPointToPointDistance(p1.head<3>(), p2.head<3>(), start_point, end_point) - seg.bounding_box.radius -
            obstacle.bounding_box.getRadius(t_radius);

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE && t2 == robot_misc::BoundingBoxType::BOX)
    {
        // sphere - box
        Eigen::Matrix<double, 4, 1> p1, p2, p3, q;
        T2.noalias() = obstacle.state.getPose(t) * obstacle.bounding_box.T;

        q.noalias() = (full_direct_kinematics[j] * seg.bounding_box.T).block<4, 1>(0, 3);

        p1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle.bounding_box.length_x;
        p2.noalias() = T2 * end;
        end(0)       = 0.0;
        end(1)       = obstacle.bounding_box.length_y;
        p3.noalias() = T2 * end;

        d = getPointToRectangleDistance(q.head<3>(), p1.head<3>(), p2.head<3>(), p3.head<3>(), start_point, end_point) - seg.bounding_box.radius -
            obstacle.bounding_box.getRadius(t_radius);

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::CYLINDER && t2 == robot_misc::BoundingBoxType::BOX)
    {

        // cylinder - box
        Eigen::Matrix<double, 4, 1> p1, p2, q1, q2, q3;
        T1.noalias() = full_direct_kinematics[j] * seg.bounding_box.T;
        T2.noalias() = obstacle.state.getPose(t) * obstacle.bounding_box.T;

        p1           = T1.block<4, 1>(0, 3);
        end(0)       = seg.bounding_box.length_x;
        p2.noalias() = T1 * end;

        q1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle.bounding_box.length_x;
        q2.noalias() = T2 * end;
        end(0)       = 0.0;
        end(1)       = obstacle.bounding_box.length_y;
        q3.noalias() = T2 * end;

        d = getLineToRectangleDistance(p1.head<3>(), p2.head<3>(), q1.head<3>(), q2.head<3>(), q3.head<3>(), start_point, end_point) -
            seg.bounding_box.radius - obstacle.bounding_box.getRadius(t_radius);
        // TODO fix problems with this distance function or the MB version and use it instead of the GTE version
        // d = getLineToRectangleDistance(p1.head<3>(), p2.head<3>(), q1.head<3>(), q2.head<3>(), q3.head<3>(), start_point, end_point) -
        // seg.bounding_box.radius - obstacle.bounding_box.radius;
        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE && t2 == robot_misc::BoundingBoxType::EBOX)
    {
        // sphere - box
        Eigen::Matrix<double, 4, 1> p1, p2, p3, q;
        T2.noalias() = obstacle.state.getPose(t) * obstacle.bounding_box.T;

        q.noalias() = (full_direct_kinematics[j] * seg.bounding_box.T).block<4, 1>(0, 3);

        p1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle.bounding_box.length_x;
        p2.noalias() = T2 * end;
        end(0)       = 0.0;
        end(1)       = obstacle.bounding_box.length_y;
        p3.noalias() = T2 * end;

        d = getPointToExtrudedRectangleDistance(q.head<3>(), p1.head<3>(), p2.head<3>(), p3.head<3>(), start_point, end_point) -
            seg.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::CYLINDER && t2 == robot_misc::BoundingBoxType::EBOX)
    {
        // cylinder - box
        Eigen::Matrix<double, 4, 1> p1, p2, q1, q2, q3;
        T1.noalias() = full_direct_kinematics[j] * seg.bounding_box.T;
        T2.noalias() = obstacle.state.getPose(t) * obstacle.bounding_box.T;

        p1           = T1.block<4, 1>(0, 3);
        end(0)       = seg.bounding_box.length_x;
        p2.noalias() = T1 * end;

        q1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle.bounding_box.length_x;
        q2.noalias() = T2 * end;
        end(0)       = 0.0;
        end(1)       = obstacle.bounding_box.length_y;
        q3.noalias() = T2 * end;

        d = getLineToExtrudedRectangleDistance(p1.head<3>(), p2.head<3>(), q1.head<3>(), q2.head<3>(), q3.head<3>(), start_point, end_point) -
            seg.bounding_box.radius;

        return true;
    }
    return false;
}

bool RobotCollision::getDistanceToUncertaintyObstacle(double& d, int j, const robot_misc::Obstacle& obstacle, double t, int iterator) const
{
    Eigen::Matrix<double, 3, 1> start_point, end_point;
    return getDistanceToUncertaintyObstacle(d, j, obstacle, start_point, end_point, t, iterator);
}

bool RobotCollision::getDistanceToUncertaintyObstacle(double& d, int j, const robot_misc::Obstacle& obstacle,
                                                      Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                      Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t, int iterator) const
{
    const robot_misc::Segment& seg = _segment_structs[j];

    // check if segment bbox is inactive
    if (!seg.bounding_box.active) return false;

    const std::vector<Eigen::Matrix<double, 4, 4>>& full_direct_kinematics = _robot_kinematic->getForwardKinematicChain();

    robot_misc::BoundingBoxType t1, t2;
    t1 = seg.bounding_box.type;
    t2 = obstacle.bounding_box.type;

    Eigen::Matrix<double, 4, 1> end;
    end << 0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4d T1, T2;

    if (t1 == robot_misc::BoundingBoxType::CYLINDER && t2 == robot_misc::BoundingBoxType::CYLINDER)
    {
        // cylinder - cylinder
        Eigen::Matrix<double, 4, 1> p1, p2, q1, q2;
        T1.noalias() = full_direct_kinematics[j] * seg.bounding_box.T;
        T2.noalias() = obstacle.uncertainty_states.at(iterator).getPose(t) * obstacle.bounding_box.T;

        p1 = T1.block<4, 1>(0, 3);

        end(0)       = seg.bounding_box.length_x;
        p2.noalias() = T1 * end;
        q1           = T2.block<4, 1>(0, 3);

        end(0)       = obstacle.bounding_box.length_x;
        q2.noalias() = T2 * end;

        d = getLineToLineDistance(p1.head<3>(), p2.head<3>(), q1.head<3>(), q2.head<3>(), start_point, end_point) - seg.bounding_box.radius -
            obstacle.bounding_box.getRadius(t);

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::CYLINDER && t2 == robot_misc::BoundingBoxType::SPHERE)
    {
        // cylinder - spehre
        Eigen::Matrix<double, 4, 1> p1, p2, q;
        T1.noalias() = full_direct_kinematics[j] * seg.bounding_box.T;

        p1 = T1.block<4, 1>(0, 3);

        end(0)       = seg.bounding_box.length_x;
        p2.noalias() = T1 * end;
        q.noalias()  = (obstacle.uncertainty_states.at(iterator).getPose(t) * obstacle.bounding_box.T).block<4, 1>(0, 3);

        d = getPointToLineDistance(q.head<3>(), p1.head<3>(), p2.head<3>(), end_point, start_point) - seg.bounding_box.radius -
            obstacle.bounding_box.getRadius(t);

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE && t2 == robot_misc::BoundingBoxType::CYLINDER)
    {
        // sphere - cylinder
        Eigen::Matrix<double, 4, 1> p, q1, q2;
        T2.noalias() = obstacle.uncertainty_states.at(iterator).getPose(t) * obstacle.bounding_box.T;

        p  = (full_direct_kinematics[j] * seg.bounding_box.T).block<4, 1>(0, 3);
        q1 = T2.block<4, 1>(0, 3);

        end(0)       = obstacle.bounding_box.length_x;
        q2.noalias() = T2 * end;

        d = getPointToLineDistance(p.head<3>(), q1.head<3>(), q2.head<3>(), start_point, end_point) - seg.bounding_box.radius -
            obstacle.bounding_box.getRadius(t);

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE && t2 == robot_misc::BoundingBoxType::SPHERE)
    {
        // sphere - sphere
        Eigen::Matrix<double, 4, 1> p1, p2;

        p1.noalias() = (full_direct_kinematics[j] * seg.bounding_box.T).block<4, 1>(0, 3);
        p2.noalias() = (obstacle.uncertainty_states.at(iterator).getPose(t) * obstacle.bounding_box.T).block<4, 1>(0, 3);

        d = getPointToPointDistance(p1.head<3>(), p2.head<3>(), start_point, end_point) - seg.bounding_box.radius -
            obstacle.bounding_box.getRadius(t);

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE && t2 == robot_misc::BoundingBoxType::BOX)
    {
        // sphere - box
        Eigen::Matrix<double, 4, 1> p1, p2, p3, q;
        T2.noalias() = obstacle.uncertainty_states.at(iterator).getPose(t) * obstacle.bounding_box.T;

        q.noalias() = (full_direct_kinematics[j] * seg.bounding_box.T).block<4, 1>(0, 3);

        p1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle.bounding_box.length_x;
        p2.noalias() = T2 * end;
        end(0)       = 0.0;
        end(1)       = obstacle.bounding_box.length_y;
        p3.noalias() = T2 * end;

        d = getPointToRectangleDistance(q.head<3>(), p1.head<3>(), p2.head<3>(), p3.head<3>(), start_point, end_point) - seg.bounding_box.radius -
            obstacle.bounding_box.getRadius(t);

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::CYLINDER && t2 == robot_misc::BoundingBoxType::BOX)
    {

        // cylinder - box
        Eigen::Matrix<double, 4, 1> p1, p2, q1, q2, q3;
        T1.noalias() = full_direct_kinematics[j] * seg.bounding_box.T;
        T2.noalias() = obstacle.uncertainty_states.at(iterator).getPose(t) * obstacle.bounding_box.T;

        p1           = T1.block<4, 1>(0, 3);
        end(0)       = seg.bounding_box.length_x;
        p2.noalias() = T1 * end;

        q1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle.bounding_box.length_x;
        q2.noalias() = T2 * end;
        end(0)       = 0.0;
        end(1)       = obstacle.bounding_box.length_y;
        q3.noalias() = T2 * end;
        d            = getLineToRectangleDistance(p1.head<3>(), p2.head<3>(), q1.head<3>(), q2.head<3>(), q3.head<3>(), start_point, end_point) -
            seg.bounding_box.radius - obstacle.bounding_box.getRadius(t);

        // TODO fix problems with this distance function or the MB version and use it instead of the GTE version
        // d = getLineToRectangleDistance(p1.head<3>(), p2.head<3>(), q1.head<3>(), q2.head<3>(), q3.head<3>(), start_point, end_point) -
        // seg.bounding_box.radius - obstacle.bounding_box.radius;
        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE && t2 == robot_misc::BoundingBoxType::EBOX)
    {
        // sphere - box
        Eigen::Matrix<double, 4, 1> p1, p2, p3, q;
        T2.noalias() = obstacle.uncertainty_states.at(iterator).getPose(t) * obstacle.bounding_box.T;

        q.noalias() = (full_direct_kinematics[j] * seg.bounding_box.T).block<4, 1>(0, 3);

        p1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle.bounding_box.length_x;
        p2.noalias() = T2 * end;
        end(0)       = 0.0;
        end(1)       = obstacle.bounding_box.length_y;
        p3.noalias() = T2 * end;

        d = getPointToExtrudedRectangleDistance(q.head<3>(), p1.head<3>(), p2.head<3>(), p3.head<3>(), start_point, end_point) -
            seg.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::CYLINDER && t2 == robot_misc::BoundingBoxType::EBOX)
    {
        // cylinder - box
        Eigen::Matrix<double, 4, 1> p1, p2, q1, q2, q3;
        T1.noalias() = full_direct_kinematics[j] * seg.bounding_box.T;
        T2.noalias() = obstacle.uncertainty_states.at(iterator).getPose(t) * obstacle.bounding_box.T;

        p1           = T1.block<4, 1>(0, 3);
        end(0)       = seg.bounding_box.length_x;
        p2.noalias() = T1 * end;

        q1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle.bounding_box.length_x;
        q2.noalias() = T2 * end;
        end(0)       = 0.0;
        end(1)       = obstacle.bounding_box.length_y;
        q3.noalias() = T2 * end;

        d = getLineToExtrudedRectangleDistance(p1.head<3>(), p2.head<3>(), q1.head<3>(), q2.head<3>(), q3.head<3>(), start_point, end_point) -
            seg.bounding_box.radius;

        return true;
    }
    return false;
}

bool RobotCollision::getDistanceToPlane(double& d, int j, const robot_misc::Plane& plane) const
{
    Eigen::Matrix<double, 3, 1> start_point, end_point;
    return getDistanceToPlane(d, j, plane, start_point, end_point);
}

bool RobotCollision::getDistanceToPlane(double& d, int j, const robot_misc::Plane& plane, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                        Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point) const
{
    const robot_misc::Segment& seg = _segment_structs[j];

    // check if segment bbox is inactive
    if (!seg.bounding_box.active) return false;

    const std::vector<Eigen::Matrix<double, 4, 4>>& full_direct_kinematics = _robot_kinematic->getForwardKinematicChain();

    robot_misc::BoundingBoxType t1 = seg.bounding_box.type;
    Eigen::Matrix<double, 4, 1> end;
    end << 0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4d T1;

    if (t1 == robot_misc::BoundingBoxType::CYLINDER)
    {
        // cylinder - plane
        Eigen::Matrix<double, 4, 1> p1, p2;
        T1.noalias() = full_direct_kinematics[j] * seg.bounding_box.T;

        p1 = T1.block<4, 1>(0, 3);

        end(0)       = seg.bounding_box.length_x;
        p2.noalias() = T1 * end;

        d = getLineToPlaneDistance(p1.head<3>(), p2.head<3>(), plane.q, plane.n, start_point, end_point) - seg.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE)
    {
        // sphere - plane
        Eigen::Matrix<double, 4, 1> p;

        p.noalias() = (full_direct_kinematics[j] * seg.bounding_box.T).block<4, 1>(0, 3);

        d = getPointToPlaneDistance(p.head<3>(), plane.q, plane.n, start_point, end_point) - seg.bounding_box.radius;

        return true;
    }
    return false;
}

bool RobotCollision::getDistanceBetweenObstacles(double& d, const robot_misc::Obstacle& obstacle1, const robot_misc::Obstacle& obstacle2, double t)
{
    Eigen::Matrix<double, 3, 1> start_point, end_point;
    return getDistanceBetweenObstacles(d, obstacle1, obstacle2, start_point, end_point);
}

bool RobotCollision::getDistanceBetweenObstacles(double& d, const robot_misc::Obstacle& obstacle1, const robot_misc::Obstacle& obstacle2,
                                                 Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                 Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t)
{
    // obstacle bbs must be active
    if (!obstacle1.bounding_box.active || !obstacle2.bounding_box.active) return false;

    robot_misc::BoundingBoxType t1, t2;
    t1 = obstacle1.bounding_box.type;
    t2 = obstacle2.bounding_box.type;

    Eigen::Matrix<double, 4, 1> end;
    end << 0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4d T1, T2;

    if (t1 == robot_misc::BoundingBoxType::CYLINDER && t2 == robot_misc::BoundingBoxType::CYLINDER)
    {
        // cylinder - cylinder
        Eigen::Matrix<double, 4, 1> p1, p2, q1, q2;
        T1.noalias() = obstacle1.state.getPose(t) * obstacle1.bounding_box.T;
        T2.noalias() = obstacle2.state.getPose(t) * obstacle2.bounding_box.T;

        p1           = T1.block<4, 1>(0, 3);
        end(0)       = obstacle1.bounding_box.length_x;
        p2.noalias() = T1 * end;

        q1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle2.bounding_box.length_x;
        q2.noalias() = T2 * end;

        d = getLineToLineDistance(p1.head<3>(), p2.head<3>(), q1.head<3>(), q2.head<3>(), start_point, end_point) - obstacle1.bounding_box.radius -
            obstacle2.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::CYLINDER && t2 == robot_misc::BoundingBoxType::SPHERE)
    {
        // cylinder - spehre
        Eigen::Matrix<double, 4, 1> p1, p2, q;
        T1.noalias() = obstacle1.state.getPose(t) * obstacle1.bounding_box.T;

        p1           = T1.block<4, 1>(0, 3);
        end(0)       = obstacle1.bounding_box.length_x;
        p2.noalias() = T1 * end;

        q.noalias() = (obstacle2.state.getPose(t) * obstacle2.bounding_box.T).block<4, 1>(0, 3);

        d = getPointToLineDistance(q.head<3>(), p1.head<3>(), p2.head<3>(), end_point, start_point) - obstacle1.bounding_box.radius -
            obstacle2.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE && t2 == robot_misc::BoundingBoxType::CYLINDER)
    {
        // sphere - cylinder
        Eigen::Matrix<double, 4, 1> p, q1, q2;
        T2.noalias() = obstacle2.state.getPose(t) * obstacle2.bounding_box.T;

        p.noalias() = (obstacle1.state.getPose(t) * obstacle1.bounding_box.T).block<4, 1>(0, 3);

        q1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle2.bounding_box.length_x;
        q2.noalias() = T2 * end;

        d = getPointToLineDistance(p.head<3>(), q1.head<3>(), q2.head<3>(), start_point, end_point) - obstacle1.bounding_box.radius -
            obstacle2.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE && t2 == robot_misc::BoundingBoxType::SPHERE)
    {
        // sphere - sphere
        Eigen::Matrix<double, 4, 1> p1, p2;

        p1.noalias() = (obstacle1.state.getPose(t) * obstacle1.bounding_box.T).block<4, 1>(0, 3);
        p2.noalias() = (obstacle2.state.getPose(t) * obstacle2.bounding_box.T).block<4, 1>(0, 3);

        d = getPointToPointDistance(p1.head<3>(), p2.head<3>(), start_point, end_point) - obstacle1.bounding_box.radius -
            obstacle2.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE && t2 == robot_misc::BoundingBoxType::BOX)
    {
        // sphere - box
        Eigen::Matrix<double, 4, 1> p1, p2, p3, q;
        T2.noalias() = obstacle2.state.getPose(t) * obstacle2.bounding_box.T;

        q.noalias() = (obstacle1.state.getPose(t) * obstacle1.bounding_box.T).block<4, 1>(0, 3);

        p1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle2.bounding_box.length_x;
        p2.noalias() = T2 * end;
        end(0)       = 0.0;
        end(1)       = obstacle2.bounding_box.length_y;
        p3.noalias() = T2 * end;

        d = getPointToRectangleDistance(q.head<3>(), p1.head<3>(), p2.head<3>(), p3.head<3>(), start_point, end_point) -
            obstacle1.bounding_box.radius - obstacle2.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::BOX && t2 == robot_misc::BoundingBoxType::SPHERE)
    {
        // box - sphere
        Eigen::Matrix<double, 4, 1> p, q1, q2, q3;
        T1.noalias() = obstacle1.state.getPose(t) * obstacle1.bounding_box.T;

        p.noalias() = (obstacle2.state.getPose(t) * obstacle2.bounding_box.T).block<4, 1>(0, 3);

        q1           = T1.block<4, 1>(0, 3);
        end(0)       = obstacle1.bounding_box.length_x;
        q2.noalias() = T1 * end;
        end(0)       = 0.0;
        end(1)       = obstacle1.bounding_box.length_y;
        q3.noalias() = T1 * end;

        d = getPointToRectangleDistance(p.head<3>(), q1.head<3>(), q2.head<3>(), q3.head<3>(), start_point, end_point) -
            obstacle1.bounding_box.radius - obstacle2.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::CYLINDER && t2 == robot_misc::BoundingBoxType::BOX)
    {
        // cylinder - box
        Eigen::Matrix<double, 4, 1> p1, p2, q1, q2, q3;
        T1.noalias() = obstacle1.state.getPose(t) * obstacle1.bounding_box.T;
        T2.noalias() = obstacle2.state.getPose(t) * obstacle2.bounding_box.T;

        p1           = T1.block<4, 1>(0, 3);
        end(0)       = obstacle1.bounding_box.length_x;
        p2.noalias() = T1 * end;

        q1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle2.bounding_box.length_x;
        q2.noalias() = T2 * end;
        end(0)       = 0.0;
        end(1)       = obstacle2.bounding_box.length_y;
        q3.noalias() = T2 * end;

        d = getLineToRectangleDistance(p1.head<3>(), p2.head<3>(), q1.head<3>(), q2.head<3>(), q3.head<3>(), start_point, end_point) -
            obstacle1.bounding_box.radius - obstacle2.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::BOX && t2 == robot_misc::BoundingBoxType::CYLINDER)
    {
        // box - cylinder
        Eigen::Matrix<double, 4, 1> p1, p2, q1, q2, q3;
        T1.noalias() = obstacle1.state.getPose(t) * obstacle1.bounding_box.T;
        T2.noalias() = obstacle2.state.getPose(t) * obstacle2.bounding_box.T;

        p1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle2.bounding_box.length_x;
        p2.noalias() = T2 * end;

        q1           = T1.block<4, 1>(0, 3);
        end(0)       = obstacle1.bounding_box.length_x;
        q2.noalias() = T1 * end;
        end(0)       = 0.0;
        end(1)       = obstacle1.bounding_box.length_y;
        q3.noalias() = T1 * end;
        d            = getLineToRectangleDistance(p1.head<3>(), p2.head<3>(), q1.head<3>(), q2.head<3>(), q3.head<3>(), start_point, end_point) -
            obstacle1.bounding_box.radius - obstacle2.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE && t2 == robot_misc::BoundingBoxType::EBOX)
    {
        // sphere - box
        Eigen::Matrix<double, 4, 1> p1, p2, p3, q;
        T2.noalias() = obstacle2.state.getPose(t) * obstacle2.bounding_box.T;

        q.noalias() = (obstacle1.state.getPose(t) * obstacle1.bounding_box.T).block<4, 1>(0, 3);

        p1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle2.bounding_box.length_x;
        p2.noalias() = T2 * end;
        end(0)       = 0.0;
        end(1)       = obstacle2.bounding_box.length_y;
        p3.noalias() = T2 * end;

        d = getPointToExtrudedRectangleDistance(q.head<3>(), p1.head<3>(), p2.head<3>(), p3.head<3>(), start_point, end_point) -
            obstacle1.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::EBOX && t2 == robot_misc::BoundingBoxType::SPHERE)
    {
        // box - sphere
        Eigen::Matrix<double, 4, 1> p, q1, q2, q3;
        T1.noalias() = obstacle1.state.getPose(t) * obstacle1.bounding_box.T;

        p.noalias() = (obstacle2.state.getPose(t) * obstacle2.bounding_box.T).block<4, 1>(0, 3);

        q1           = T1.block<4, 1>(0, 3);
        end(0)       = obstacle1.bounding_box.length_x;
        q2.noalias() = T1 * end;
        end(0)       = 0.0;
        end(1)       = obstacle1.bounding_box.length_y;
        q3.noalias() = T1 * end;

        d = getPointToExtrudedRectangleDistance(p.head<3>(), q1.head<3>(), q2.head<3>(), q3.head<3>(), start_point, end_point) -
            obstacle2.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::CYLINDER && t2 == robot_misc::BoundingBoxType::EBOX)
    {
        // cylinder - box
        Eigen::Matrix<double, 4, 1> p1, p2, q1, q2, q3;
        T1.noalias() = obstacle1.state.getPose(t) * obstacle1.bounding_box.T;
        T2.noalias() = obstacle2.state.getPose(t) * obstacle2.bounding_box.T;

        p1           = T1.block<4, 1>(0, 3);
        end(0)       = obstacle1.bounding_box.length_x;
        p2.noalias() = T1 * end;

        q1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle2.bounding_box.length_x;
        q2.noalias() = T2 * end;
        end(0)       = 0.0;
        end(1)       = obstacle2.bounding_box.length_y;
        q3.noalias() = T2 * end;

        d = getLineToExtrudedRectangleDistance(p1.head<3>(), p2.head<3>(), q1.head<3>(), q2.head<3>(), q3.head<3>(), start_point, end_point) -
            obstacle1.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::EBOX && t2 == robot_misc::BoundingBoxType::CYLINDER)
    {
        // box - cylinder
        Eigen::Matrix<double, 4, 1> p1, p2, q1, q2, q3;
        T1.noalias() = obstacle1.state.getPose(t) * obstacle1.bounding_box.T;
        T2.noalias() = obstacle2.state.getPose(t) * obstacle2.bounding_box.T;

        p1           = T2.block<4, 1>(0, 3);
        end(0)       = obstacle2.bounding_box.length_x;
        p2.noalias() = T2 * end;

        q1           = T1.block<4, 1>(0, 3);
        end(0)       = obstacle1.bounding_box.length_x;
        q2.noalias() = T1 * end;
        end(0)       = 0.0;
        end(1)       = obstacle1.bounding_box.length_y;
        q3.noalias() = T1 * end;

        d = getLineToExtrudedRectangleDistance(p1.head<3>(), p2.head<3>(), q1.head<3>(), q2.head<3>(), q3.head<3>(), start_point, end_point) -
            obstacle2.bounding_box.radius;

        return true;
    }
    return false;
}

bool RobotCollision::getDistanceBetweenPlaneObstacle(double& d, const robot_misc::Plane& plane, const robot_misc::Obstacle& obstacle, double t)
{
    Eigen::Matrix<double, 3, 1> start_point, end_point;
    return getDistanceBetweenPlaneObstacle(d, plane, obstacle, start_point, end_point);
}

bool RobotCollision::getDistanceBetweenPlaneObstacle(double& d, const robot_misc::Plane& plane, const robot_misc::Obstacle& obstacle,
                                                     Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                     Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t)
{
    // obstacle bb must be active
    if (!obstacle.bounding_box.active) return false;

    robot_misc::BoundingBoxType t1;
    t1 = obstacle.bounding_box.type;

    Eigen::Matrix<double, 4, 1> end;
    end << 0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4d T1;

    if (t1 == robot_misc::BoundingBoxType::CYLINDER)
    {
        // cylinder - plane
        Eigen::Matrix<double, 4, 1> p1, p2;
        T1.noalias() = obstacle.state.getPose(t) * obstacle.bounding_box.T;

        p1           = T1.block<4, 1>(0, 3);
        end(0)       = obstacle.bounding_box.length_x;
        p2.noalias() = T1 * end;

        d = getLineToPlaneDistance(p1.head<3>(), p2.head<3>(), plane.q, plane.n, start_point, end_point) - obstacle.bounding_box.radius;

        return true;
    }
    else if (t1 == robot_misc::BoundingBoxType::SPHERE)
    {
        // sphere - plane
        Eigen::Matrix<double, 4, 1> p;

        p.noalias() = (obstacle.state.getPose(t) * obstacle.bounding_box.T).block<4, 1>(0, 3);

        d = getPointToPlaneDistance(p.head<3>(), plane.q, plane.n, start_point, end_point) - obstacle.bounding_box.radius;

        return true;
    }
    return false;
}

bool RobotCollision::getDistanceBetweenHumanObstacle(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, const robot_misc::Obstacle& obstacle,
                                                     const robot_misc::Human& human, double t)
{
    Eigen::Matrix<double, 3, robot_misc::Human::_body_parts_size> start_point, end_point;
    return getDistanceBetweenHumanObstacle(d, obstacle, human, start_point, end_point, t);
}

bool RobotCollision::getDistanceBetweenHumanObstacle(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, const robot_misc::Obstacle& obstacle,
                                                     const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                                     Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t)
{
    bool success = true;

    for (int i = 0; i < human._body_parts_size; i++)
    {
        success &=
            getDistanceBetweenObstacles(d(i), obstacle, human._body_parts.find(human._distance_body_part_names[static_cast<size_t>(i)])->second,
                                        start_points.col(i), end_points.col(i), t);
    }

    return success;
}

bool RobotCollision::getMinDistanceBetweenHumanObstacle(double& d, const robot_misc::Obstacle& obstacle, const robot_misc::Human& human, double t)
{
    Eigen::Matrix<double, 3, 1> start_point, end_point;
    return getMinDistanceBetweenHumanObstacle(d, obstacle, human, start_point, end_point, t);
}

bool RobotCollision::getMinDistanceBetweenHumanObstacle(double& d, const robot_misc::Obstacle& obstacle, const robot_misc::Human& human,
                                                        Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                        Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t)
{
    Eigen::Matrix<double, robot_misc::Human::_body_parts_size, 1> distances;
    Eigen::Matrix<double, 3, robot_misc::Human::_body_parts_size> start_points, end_points;

    bool success = getDistanceBetweenHumanObstacle(distances, obstacle, human, start_points, end_points, t);

    Eigen::MatrixXd::Index minRow, minCol;
    d = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return success;
}

double RobotCollision::getMinSelfCollisionDistance() const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_self_collision_distance_dimension, 1);
    getSelfCollisionDistances(distances);
    return distances.minCoeff();
}

double RobotCollision::getMinSelfCollisionDistance(Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                   Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_self_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _self_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _self_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getSelfCollisionDistances(distances, start_points, end_points);
    double dist = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinObstacleDistance(const robot_misc::Obstacle& obstacle, double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_obstacle_distance_dimension, 1);
    getObstacleDistances(obstacle, distances, t);
    return distances.minCoeff();
}

double RobotCollision::getMinObstacleDistance(const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                              Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_obstacle_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _obstacle_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _obstacle_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getObstacleDistances(obstacle, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

void RobotCollision::getHumanDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    getHumanHeadDistances(human, d.segment(0, _human_collision_distance_dimension), t);
    getHumanThorsoDistances(human, d.segment(_human_collision_distance_dimension, _human_collision_distance_dimension), t);
    //    getHumanPelvisDistances(human, d.segment(2 * _human_collision_distance_dimension, _human_collision_distance_dimension), t);
    getHumanUpperArmRightDistances(human, d.segment(2 * _human_collision_distance_dimension, _human_collision_distance_dimension), t);
    getHumanUpperArmLeftDistances(human, d.segment(3 * _human_collision_distance_dimension, _human_collision_distance_dimension), t);
    getHumanLowerArmRightDistances(human, d.segment(4 * _human_collision_distance_dimension, _human_collision_distance_dimension), t);
    getHumanLowerArmLeftDistances(human, d.segment(5 * _human_collision_distance_dimension, _human_collision_distance_dimension), t);
    //    getHumanHandRightDistances(human, d.segment(7 * _human_collision_distance_dimension, _human_collision_distance_dimension), t);
    //    getHumanHandLeftDistances(human, d.segment(8 * _human_collision_distance_dimension, _human_collision_distance_dimension), t);
    getHumanLegDistances(human, d.segment(6 * _human_collision_distance_dimension, _human_collision_distance_dimension), t);
}

void RobotCollision::getHumanDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                                     double t_radius) const
{
    getHumanHeadDistancesRadiusIncrease(human, d.segment(0, _human_collision_distance_dimension), t, t_radius);
    getHumanThorsoDistancesRadiusIncrease(human, d.segment(_human_collision_distance_dimension, _human_collision_distance_dimension), t, t_radius);
    //    getHumanPelvisDistances(human, d.segment(2 * _human_collision_distance_dimension, _human_collision_distance_dimension), t,t);
    getHumanUpperArmRightDistancesRadiusIncrease(human, d.segment(2 * _human_collision_distance_dimension, _human_collision_distance_dimension), t,
                                                 t_radius);
    getHumanUpperArmLeftDistancesRadiusIncrease(human, d.segment(3 * _human_collision_distance_dimension, _human_collision_distance_dimension), t,
                                                t_radius);
    getHumanLowerArmRightDistancesRadiusIncrease(human, d.segment(4 * _human_collision_distance_dimension, _human_collision_distance_dimension), t,
                                                 t_radius);
    getHumanLowerArmLeftDistancesRadiusIncrease(human, d.segment(5 * _human_collision_distance_dimension, _human_collision_distance_dimension), t,
                                                t_radius);
    //    getHumanHandRightDistances(human, d.segment(7 * _human_collision_distance_dimension, _human_collision_distance_dimension), t,t);
    //    getHumanHandLeftDistances(human, d.segment(8 * _human_collision_distance_dimension, _human_collision_distance_dimension), t,t);
    getHumanLegDistancesRadiusIncrease(human, d.segment(6 * _human_collision_distance_dimension, _human_collision_distance_dimension), t, t_radius);
}

void RobotCollision::getHumanDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                       Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                       double t) const
{
    getHumanHeadDistances(human, d.segment(0, _human_collision_distance_dimension), start_points.block(0, 0, 3, _human_collision_distance_dimension),
                          end_points.block(0, 0, 3, _human_collision_distance_dimension), t);

    getHumanThorsoDistances(human, d.segment(_human_collision_distance_dimension, _human_collision_distance_dimension),
                            start_points.block(0, _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
                            end_points.block(0, _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);

    //    getHumanPelvisDistances(human, d.segment(2 * _human_collision_distance_dimension, _human_collision_distance_dimension),
    //                            start_points.block(0, 2 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
    //                            end_points.block(0, 2 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);

    getHumanUpperArmRightDistances(human, d.segment(2 * _human_collision_distance_dimension, _human_collision_distance_dimension),
                                   start_points.block(0, 2 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
                                   end_points.block(0, 2 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);

    getHumanUpperArmLeftDistances(human, d.segment(3 * _human_collision_distance_dimension, _human_collision_distance_dimension),
                                  start_points.block(0, 3 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
                                  end_points.block(0, 3 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);

    getHumanLowerArmRightDistances(human, d.segment(4 * _human_collision_distance_dimension, _human_collision_distance_dimension),
                                   start_points.block(0, 4 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
                                   end_points.block(0, 4 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);

    getHumanLowerArmLeftDistances(human, d.segment(5 * _human_collision_distance_dimension, _human_collision_distance_dimension),
                                  start_points.block(0, 5 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
                                  end_points.block(0, 5 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);

    //    getHumanHandRightDistances(human, d.segment(7 * _human_collision_distance_dimension, _human_collision_distance_dimension),
    //                               start_points.block(0, 7 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
    //                               end_points.block(0, 7 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);

    //    getHumanHandLeftDistances(human, d.segment(8 * _human_collision_distance_dimension, _human_collision_distance_dimension),
    //                              start_points.block(0, 8 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
    //                              end_points.block(0, 8 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);

    getHumanLegDistances(human, d.segment(6 * _human_collision_distance_dimension, _human_collision_distance_dimension),
                         start_points.block(0, 6 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
                         end_points.block(0, 6 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);
}

void RobotCollision::getMinHumanDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    d(0) = getMinHumanHeadDistance(human, t);
    d(1) = getMinHumanThorsoDistance(human, t);
    //    d(2) = getMinHumanPelvisDistance(human, t);
    d(2) = getMinHumanUpperArmRightDistance(human, t);
    d(3) = getMinHumanUpperArmLeftDistance(human, t);
    d(4) = getMinHumanLowerArmRightDistance(human, t);
    d(5) = getMinHumanLowerArmLeftDistance(human, t);
    //    d(7) = getMinHumanHandRightDistance(human, t);
    //    d(8) = getMinHumanHandLeftDistance(human, t);
    d(6) = getMinHumanLegDistance(human, t);
}

void RobotCollision::getMinHumanDistanceRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                                       double t_radius) const
{
    d(0) = getMinHumanHeadDistanceRadiusIncrease(human, t, t_radius);
    d(1) = getMinHumanThorsoDistanceRadiusIncrease(human, t, t_radius);
    //    d(2) = getMinHumanPelvisDistanceRadiusIncrease(human, t,t_radius);
    d(2) = getMinHumanUpperArmRightDistanceRadiusIncrease(human, t, t_radius);
    d(3) = getMinHumanUpperArmLeftDistanceRadiusIncrease(human, t, t_radius);
    d(4) = getMinHumanLowerArmRightDistanceRadiusIncrease(human, t, t_radius);
    d(5) = getMinHumanLowerArmLeftDistanceRadiusIncrease(human, t, t_radius);
    //    d(7) = getMinHumanHandRightDistanceRadiusIncrease(human, t,t_radius);
    //    d(8) = getMinHumanHandLeftDistanceRadiusIncrease(human, t,t_radius);
    d(6) = getMinHumanLegDistanceRadiusIncrease(human, t, t_radius);
}

void RobotCollision::getMinHumanDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                         Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                         double t) const
{
    d(0) = getMinHumanHeadDistance(human, start_points.col(0), end_points.col(0), t);
    d(1) = getMinHumanThorsoDistance(human, start_points.col(1), end_points.col(1), t);
    //    d(2) = getMinHumanPelvisDistance(human, start_points.col(2), end_points.col(2), t);
    d(2) = getMinHumanUpperArmRightDistance(human, start_points.col(2), end_points.col(2), t);
    d(3) = getMinHumanUpperArmLeftDistance(human, start_points.col(3), end_points.col(3), t);
    d(4) = getMinHumanLowerArmRightDistance(human, start_points.col(4), end_points.col(4), t);
    d(5) = getMinHumanLowerArmLeftDistance(human, start_points.col(5), end_points.col(5), t);
    //    d(7) = getMinHumanHandRightDistance(human, start_points.col(7), end_points.col(7), t);
    //    d(8) = getMinHumanHandLeftDistance(human, start_points.col(8), end_points.col(8), t);
    d(6) = getMinHumanLegDistance(human, start_points.col(6), end_points.col(6), t);
}

void RobotCollision::getMinHumanDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                         Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                         std::vector<int>& segments, double t) const
{
    d(0) = getMinHumanHeadDistance(human, start_points.col(0), end_points.col(0), segments[0], t);
    d(1) = getMinHumanThorsoDistance(human, start_points.col(1), end_points.col(1), segments[1], t);
    //    d(2) = getMinHumanPelvisDistance(human, start_points.col(2), end_points.col(2), segments[2], t);
    d(2) = getMinHumanUpperArmRightDistance(human, start_points.col(2), end_points.col(2), segments[2], t);
    d(3) = getMinHumanUpperArmLeftDistance(human, start_points.col(3), end_points.col(3), segments[3], t);
    d(4) = getMinHumanLowerArmRightDistance(human, start_points.col(4), end_points.col(4), segments[4], t);
    d(5) = getMinHumanLowerArmLeftDistance(human, start_points.col(5), end_points.col(5), segments[5], t);
    //    d(7) = getMinHumanHandRightDistance(human, start_points.col(7), end_points.col(7), segments[7], t);
    //    d(8) = getMinHumanHandLeftDistance(human, start_points.col(8), end_points.col(8), segments[8], t);
    d(6) = getMinHumanLegDistance(human, start_points.col(6), end_points.col(6), segments[6], t);
}

void RobotCollision::getHumanTrunkDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    getHumanThorsoDistances(human, d.segment(0, _human_collision_distance_dimension), t);
    //    getHumanPelvisDistances(human, d.segment(_human_collision_distance_dimension, _human_collision_distance_dimension), t);
    getHumanLegDistances(human, d.segment(_human_collision_distance_dimension, _human_collision_distance_dimension), t);
}

void RobotCollision::getHumanTrunkDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                               const int uncertainty_instance, double t) const
{
    getHumanThorsoDistancesUncertaintyInstance(human, d.segment(0, _human_collision_distance_dimension), uncertainty_instance, t);
    getHumanLegDistancesUncertaintyInstance(human, d.segment(_human_collision_distance_dimension, _human_collision_distance_dimension),
                                            uncertainty_instance, t);
}

void RobotCollision::getHumanTrunkDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                                          double t_radius) const
{
    getHumanThorsoDistancesRadiusIncrease(human, d.segment(0, _human_collision_distance_dimension), t, t_radius);
    //    getHumanPelvisDistances(human, d.segment(_human_collision_distance_dimension, _human_collision_distance_dimension), t);
    getHumanLegDistancesRadiusIncrease(human, d.segment(_human_collision_distance_dimension, _human_collision_distance_dimension), t, t_radius);
}

void RobotCollision::getHumanTrunkDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                            Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                            Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t) const
{
    getHumanThorsoDistances(human, d.segment(0, _human_collision_distance_dimension),
                            start_points.block(0, 0, 3, _human_collision_distance_dimension),
                            end_points.block(0, 0, 3, _human_collision_distance_dimension), t);

    //    getHumanPelvisDistances(human, d.segment(_human_collision_distance_dimension, _human_collision_distance_dimension),
    //                            start_points.block(0, _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
    //                            end_points.block(0, _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);

    getHumanLegDistances(human, d.segment(_human_collision_distance_dimension, _human_collision_distance_dimension),
                         start_points.block(0, 1 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
                         end_points.block(0, 1 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);
}

void RobotCollision::getHumanUncertaintyTrunkDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    int startElem = 0;
    if (human.getNumberUncertaintyNeck() > 0)
    {
        getHumanUncertaintyThorsoDistances(
            human,
            d.segment(startElem, human.getNumberUncertaintyNeck() * _human_collision_distance_dimension)
                .reshaped(_human_collision_distance_dimension,
                          d.segment(startElem, human.getNumberUncertaintyNeck() * _human_collision_distance_dimension).size() /
                              _human_collision_distance_dimension),
            t);
        startElem += human.getNumberUncertaintyRUArm() * _human_collision_distance_dimension;
    }
    //    if (human.getNumberUncertaintyLeg() > 0)
    //    {
    //        getHumanUncertaintyPelvisDistances(
    //            human,
    //            d.segment(startElem, human.getNumberUncertaintyLeg() * _human_collision_distance_dimension)
    //                .reshaped(_human_collision_distance_dimension,
    //                          d.segment(startElem, human.getNumberUncertaintyLeg() * _human_collision_distance_dimension).size() /
    //                              _human_collision_distance_dimension),
    //            t);
    //        startElem += human.getNumberUncertaintyRUArm() * _human_collision_distance_dimension;
    //    }
    if (human.getNumberUncertaintyLeg() > 0)
    {
        getHumanUncertaintyLegDistances(
            human,
            d.segment(startElem, human.getNumberUncertaintyLeg() * _human_collision_distance_dimension)
                .reshaped(_human_collision_distance_dimension,
                          d.segment(startElem, human.getNumberUncertaintyLeg() * _human_collision_distance_dimension).size() /
                              _human_collision_distance_dimension),
            t);
        startElem += human.getNumberUncertaintyRUArm() * _human_collision_distance_dimension;
    }
}

void RobotCollision::getMinHumanTrunkDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    d(0) = getMinHumanThorsoDistance(human, t);
    //    d(1) = getMinHumanPelvisDistance(human, t);
    d(1) = getMinHumanLegDistance(human, t);
}

void RobotCollision::getMinHumanTrunkDistanceUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                                 const int uncertainty_instance, double t) const
{
    d(0) = getMinHumanThorsoDistanceUncertaintyInstance(human, uncertainty_instance, t);
    d(1) = getMinHumanLegDistanceUncertaintyInstance(human, uncertainty_instance, t);
}

void RobotCollision::getMinHumanTrunkDistanceRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                                            double t_radius) const
{
    d(0) = getMinHumanThorsoDistanceRadiusIncrease(human, t, t_radius);
    //    d(1) = getMinHumanPelvisDistance(human, t);
    d(1) = getMinHumanLegDistanceRadiusIncrease(human, t, t_radius);
}

void RobotCollision::getMinHumanTrunkDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                              Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                              Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t) const
{
    d(0) = getMinHumanThorsoDistance(human, start_points.col(0), end_points.col(0), t);
    //    d(1) = getMinHumanPelvisDistance(human, start_points.col(1), end_points.col(1), t);
    d(1) = getMinHumanLegDistance(human, start_points.col(1), end_points.col(1), t);
}

void RobotCollision::getMinHumanUncertaintyTrunkDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    d(0) = (human.getNumberUncertaintyNeck() > 0) ? getMinHumanUncertaintyThorsoDistance(human, t) : 1.7976931348623157E+308;
    //    d(1) = (human.getNumberUncertaintyLeg() > 0) ? getMinHumanUncertaintyPelvisDistance(human, t) : 1.7976931348623157E+308;
    d(1) = (human.getNumberUncertaintyLeg() > 0) ? getMinHumanUncertaintyLegDistance(human, t) : 1.7976931348623157E+308;
}

void RobotCollision::getHumanLimbsDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    getHumanUpperArmRightDistances(human, d.segment(0, _human_collision_distance_dimension), t);
    getHumanUpperArmLeftDistances(human, d.segment(_human_collision_distance_dimension, _human_collision_distance_dimension), t);
    getHumanLowerArmRightDistances(human, d.segment(2 * _human_collision_distance_dimension, _human_collision_distance_dimension), t);
    getHumanLowerArmLeftDistances(human, d.segment(3 * _human_collision_distance_dimension, _human_collision_distance_dimension), t);
    //    getHumanHandRightDistances(human, d.segment(4 * _human_collision_distance_dimension, _human_collision_distance_dimension), t);
    //    getHumanHandLeftDistances(human, d.segment(5 * _human_collision_distance_dimension, _human_collision_distance_dimension), t);
}

void RobotCollision::getHumanLimbsDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                               const int uncertainty_instance, double t) const
{
    getHumanUpperArmRightDistancesUncertaintyInstance(human, d.segment(0, _human_collision_distance_dimension), uncertainty_instance, t);
    getHumanUpperArmLeftDistancesUncertaintyInstance(human, d.segment(_human_collision_distance_dimension, _human_collision_distance_dimension),
                                                     uncertainty_instance, t);
    getHumanLowerArmRightDistancesUncertaintyInstance(human, d.segment(2 * _human_collision_distance_dimension, _human_collision_distance_dimension),
                                                      uncertainty_instance, t);
    getHumanLowerArmLeftDistancesUncertaintyInstance(human, d.segment(3 * _human_collision_distance_dimension, _human_collision_distance_dimension),
                                                     uncertainty_instance, t);
}

void RobotCollision::getHumanLimbsDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                                          double t_radius) const
{
    getHumanUpperArmRightDistancesRadiusIncrease(human, d.segment(0, _human_collision_distance_dimension), t, t_radius);
    getHumanUpperArmLeftDistancesRadiusIncrease(human, d.segment(_human_collision_distance_dimension, _human_collision_distance_dimension), t,
                                                t_radius);
    getHumanLowerArmRightDistancesRadiusIncrease(human, d.segment(2 * _human_collision_distance_dimension, _human_collision_distance_dimension), t,
                                                 t_radius);
    getHumanLowerArmLeftDistancesRadiusIncrease(human, d.segment(3 * _human_collision_distance_dimension, _human_collision_distance_dimension), t,
                                                t_radius);
    //    getHumanHandRightDistances(human, d.segment(4 * _human_collision_distance_dimension, _human_collision_distance_dimension), t);
    //    getHumanHandLeftDistances(human, d.segment(5 * _human_collision_distance_dimension, _human_collision_distance_dimension), t);
}

void RobotCollision::getHumanLimbsDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                            Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                            Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t) const
{
    getHumanUpperArmRightDistances(human, d.segment(0, _human_collision_distance_dimension),
                                   start_points.block(0, 0, 3, _human_collision_distance_dimension),
                                   end_points.block(0, 0, 3, _human_collision_distance_dimension), t);

    getHumanUpperArmLeftDistances(human, d.segment(_human_collision_distance_dimension, _human_collision_distance_dimension),
                                  start_points.block(0, _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
                                  end_points.block(0, _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);

    getHumanLowerArmRightDistances(human, d.segment(2 * _human_collision_distance_dimension, _human_collision_distance_dimension),
                                   start_points.block(0, 2 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
                                   end_points.block(0, 2 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);

    getHumanLowerArmLeftDistances(human, d.segment(3 * _human_collision_distance_dimension, _human_collision_distance_dimension),
                                  start_points.block(0, 3 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
                                  end_points.block(0, 3 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);

    //    getHumanHandRightDistances(human, d.segment(4 * _human_collision_distance_dimension, _human_collision_distance_dimension),
    //                               start_points.block(0, 4 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
    //                               end_points.block(0, 4 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);

    //    getHumanHandLeftDistances(human, d.segment(5 * _human_collision_distance_dimension, _human_collision_distance_dimension),
    //                              start_points.block(0, 5 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension),
    //                              end_points.block(0, 5 * _human_collision_distance_dimension, 3, _human_collision_distance_dimension), t);
}

void RobotCollision::getHumanUncertaintyLimbsDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    int startElem = 0;
    if (human.getNumberUncertaintyRUArm() > 0)
    {
        getHumanUncertaintyUpperArmRightDistances(
            human,
            d.segment(startElem, human.getNumberUncertaintyRUArm() * _human_collision_distance_dimension)
                .reshaped(_human_collision_distance_dimension,
                          d.segment(startElem, human.getNumberUncertaintyRUArm() * _human_collision_distance_dimension).size() /
                              _human_collision_distance_dimension),
            t);
        startElem += human.getNumberUncertaintyRUArm() * _human_collision_distance_dimension;
    }
    if (human.getNumberUncertaintyLUArm() > 0)
    {
        getHumanUncertaintyUpperArmLeftDistances(
            human,
            d.segment(startElem, human.getNumberUncertaintyLUArm() * _human_collision_distance_dimension)
                .reshaped(_human_collision_distance_dimension,
                          d.segment(startElem, human.getNumberUncertaintyLUArm() * _human_collision_distance_dimension).size() /
                              _human_collision_distance_dimension),
            t);
        startElem += human.getNumberUncertaintyLUArm() * _human_collision_distance_dimension;
    }
    if (human.getNumberUncertaintyRFArm() > 0)
    {
        getHumanUncertaintyLowerArmRightDistances(
            human,
            d.segment(startElem, human.getNumberUncertaintyRFArm() * _human_collision_distance_dimension)
                .reshaped(_human_collision_distance_dimension,
                          d.segment(startElem, human.getNumberUncertaintyRFArm() * _human_collision_distance_dimension).size() /
                              _human_collision_distance_dimension),
            t);
        startElem += human.getNumberUncertaintyRFArm() * _human_collision_distance_dimension;
    }
    if (human.getNumberUncertaintyLFArm() > 0)
    {
        getHumanUncertaintyLowerArmLeftDistances(
            human,
            d.segment(startElem, human.getNumberUncertaintyLFArm() * _human_collision_distance_dimension)
                .reshaped(_human_collision_distance_dimension,
                          d.segment(startElem, human.getNumberUncertaintyLFArm() * _human_collision_distance_dimension).size() /
                              _human_collision_distance_dimension),
            t);
        startElem += human.getNumberUncertaintyLFArm() * _human_collision_distance_dimension;
    }
    //    if (human.getNumberUncertaintyRHand() > 0)
    //    {
    //        getHumanUncertaintyHandRightDistances(
    //            human,
    //            d.segment(startElem, human.getNumberUncertaintyRHand() * _human_collision_distance_dimension)
    //                .reshaped(_human_collision_distance_dimension,
    //                          d.segment(startElem, human.getNumberUncertaintyRHand() * _human_collision_distance_dimension).size() /
    //                              _human_collision_distance_dimension),
    //            t);
    //        startElem += human.getNumberUncertaintyRHand() * _human_collision_distance_dimension;
    //    }
    //    if (human.getNumberUncertaintyLHand() > 0)
    //    {
    //        getHumanUncertaintyHandLeftDistances(
    //            human,
    //            d.segment(startElem, human.getNumberUncertaintyLHand() * _human_collision_distance_dimension)
    //                .reshaped(_human_collision_distance_dimension,
    //                          d.segment(startElem, human.getNumberUncertaintyLHand() * _human_collision_distance_dimension).size() /
    //                              _human_collision_distance_dimension),
    //            t);
    //        startElem += human.getNumberUncertaintyLHand() * _human_collision_distance_dimension;
    //    }
}

void RobotCollision::getMinHumanLimbsDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    d(0) = getMinHumanUpperArmRightDistance(human, t);
    d(1) = getMinHumanUpperArmLeftDistance(human, t);
    d(2) = getMinHumanLowerArmRightDistance(human, t);
    d(3) = getMinHumanLowerArmLeftDistance(human, t);
    //    d(4) = getMinHumanHandRightDistance(human, t);
    //    d(5) = getMinHumanHandLeftDistance(human, t);
}

void RobotCollision::getMinHumanLimbsDistanceUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                                 const int uncertainty_instance, double t) const
{
    d(0) = getMinHumanUpperArmRightDistanceUncertaintyInstance(human, uncertainty_instance, t);
    d(1) = getMinHumanUpperArmLeftDistanceUncertaintyInstance(human, uncertainty_instance, t);
    d(2) = getMinHumanLowerArmRightDistanceUncertaintyInstance(human, uncertainty_instance, t);
    d(3) = getMinHumanLowerArmLeftDistanceUncertaintyInstance(human, uncertainty_instance, t);
}

void RobotCollision::getMinHumanLimbsDistanceRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                                            double t_radius) const
{
    d(0) = getMinHumanUpperArmRightDistanceRadiusIncrease(human, t, t_radius);
    d(1) = getMinHumanUpperArmLeftDistanceRadiusIncrease(human, t, t_radius);
    d(2) = getMinHumanLowerArmRightDistanceRadiusIncrease(human, t, t_radius);
    d(3) = getMinHumanLowerArmLeftDistanceRadiusIncrease(human, t, t_radius);
    //    d(4) = getMinHumanHandRightDistanceRadiusIncrease(human, t,t_radius);
    //    d(5) = getMinHumanHandLeftDistanceRadiusIncrease(human, t,t_radius);
}

void RobotCollision::getMinHumanLimbsDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                              Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                              Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t) const
{
    d(0) = getMinHumanUpperArmRightDistance(human, start_points.col(0), end_points.col(0), t);
    d(1) = getMinHumanUpperArmLeftDistance(human, start_points.col(1), end_points.col(1), t);
    d(2) = getMinHumanLowerArmRightDistance(human, start_points.col(2), end_points.col(2), t);
    d(3) = getMinHumanLowerArmLeftDistance(human, start_points.col(3), end_points.col(3), t);
    //    d(4) = getMinHumanHandRightDistance(human, start_points.col(2), end_points.col(2), t);
    //    d(5) = getMinHumanHandLeftDistance(human, start_points.col(3), end_points.col(3), t);
}

void RobotCollision::getMinHumanUncertaintyLimbsDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    d(0) = (human.getNumberUncertaintyRUArm() > 0) ? getMinHumanUncertaintyUpperArmRightDistance(human, t) : 1.7976931348623157E+308;
    d(1) = (human.getNumberUncertaintyLUArm() > 0) ? getMinHumanUncertaintyUpperArmLeftDistance(human, t) : 1.7976931348623157E+308;
    d(2) = (human.getNumberUncertaintyRFArm() > 0) ? getMinHumanUncertaintyLowerArmRightDistance(human, t) : 1.7976931348623157E+308;
    d(3) = (human.getNumberUncertaintyLFArm() > 0) ? getMinHumanUncertaintyLowerArmLeftDistance(human, t) : 1.7976931348623157E+308;
    //    d(4) = (human.getNumberUncertaintyRHand() > 0) ? getMinHumanUncertaintyHandRightDistance(human, t) : 1.7976931348623157E+308;
    //    d(5) = (human.getNumberUncertaintyLHand() > 0) ? getMinHumanUncertaintyHandLeftDistance(human, t) : 1.7976931348623157E+308;
}

bool RobotCollision::getHumanHeadDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    return getObstacleDistances(human.getBodyPart("Head"), d, t, 0);
}

bool RobotCollision::getHumanHeadDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                              const int uncertainty_instance, double t) const
{
    for (int var = 0; var < human.getNumberUncertaintyHead(); ++var)
    {
        if (human.getBodyPart("Head").uncertainty_states.at(static_cast<size_t>(var)).getUncertaintyInstance() == uncertainty_instance)
        {
            // ROS_WARN("Use MS Uncertainty Distance for Head");
            return getUncertaintyObstacleDistances(human.getBodyPart("Head"), d, t, var);
        }
    }
    return getObstacleDistances(human.getBodyPart("Head"), d, t, 0);
}

bool RobotCollision::getHumanHeadDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                                         double t_radius) const
{
    return getObstacleDistances(human.getBodyPart("Head"), d, t, t_radius);
}

bool RobotCollision::getHumanHeadDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                           Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                           double t) const
{
    return getObstacleDistances(human.getBodyPart("Head"), d, start_points, end_points, t);
}

bool RobotCollision::getHumanUncertaintyHeadDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    bool success = false;
    for (int var = 0; var < human.getNumberUncertaintyHead(); ++var)
    {
        success = getUncertaintyObstacleDistances(human.getBodyPart("Head"),
                                                  d.segment(var * _human_collision_distance_dimension, _human_collision_distance_dimension), t, var);
    }
    return success;
}

double RobotCollision::getMinHumanHeadDistance(const robot_misc::Human& human, double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanHeadDistancesUncertaintyInstance(human, distances, 0, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanHeadDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance, double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanHeadDistancesUncertaintyInstance(human, distances, uncertainty_instance, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanHeadDistanceRadiusIncrease(const robot_misc::Human& human, double t, double t_radius) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanHeadDistancesRadiusIncrease(human, distances, t, t_radius);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanUncertaintyHeadDistance(const robot_misc::Human& human, double t) const
{
    if (human.getNumberUncertaintyHead() > 0)
    {
        Eigen::Matrix<double, -1, 1> distances =
            Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension * human.getNumberUncertaintyHead());
        getHumanUncertaintyHeadDistances(human, distances, t);
        return distances.minCoeff();
    }
    else
    {
        ROS_ERROR("robot_collision: Called uncertainty distance function without uncertainty");
        return -1;
    }
}

double RobotCollision::getMinHumanHeadDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                               Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanHeadDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanHeadDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                               Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanHeadDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    segment     = minRow + 2;
    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

bool RobotCollision::getHumanThorsoDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    return getObstacleDistances(human.getBodyPart("Neck"), d, t);
}

bool RobotCollision::getHumanThorsoDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                                const int uncertainty_instance, double t) const
{
    for (int var = 0; var < human.getNumberUncertaintyNeck(); ++var)
    {
        if (human.getBodyPart("Neck").uncertainty_states.at(static_cast<size_t>(var)).getUncertaintyInstance() == uncertainty_instance)
        {
            // ROS_WARN("Use MS Uncertainty Distance for Neck");
            return getUncertaintyObstacleDistances(human.getBodyPart("Neck"), d, t, var);
        }
    }
    return getObstacleDistances(human.getBodyPart("Neck"), d, t, 0);
}

bool RobotCollision::getHumanThorsoDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                                           double t_radius) const
{
    return getObstacleDistances(human.getBodyPart("Neck"), d, t, t_radius);
}

bool RobotCollision::getHumanThorsoDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                             Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                             Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t) const
{
    return getObstacleDistances(human.getBodyPart("Neck"), d, start_points, end_points, t);
}

bool RobotCollision::getHumanUncertaintyThorsoDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d, double t) const
{
    bool success = false;
    for (int var = 0; var < human.getNumberUncertaintyNeck(); ++var)
    {
        success = getUncertaintyObstacleDistances(human.getBodyPart("Neck"), d.col(var), t, var);
    }
    return success;
}

double RobotCollision::getMinHumanThorsoDistance(const robot_misc::Human& human, double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanThorsoDistances(human, distances, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanThorsoDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance, double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanThorsoDistancesUncertaintyInstance(human, distances, uncertainty_instance, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanThorsoDistanceRadiusIncrease(const robot_misc::Human& human, double t, double t_radius) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanThorsoDistancesRadiusIncrease(human, distances, t, t_radius);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanThorsoDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                 Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanThorsoDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanThorsoDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                 Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanThorsoDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    segment     = minRow + 2;
    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanUncertaintyThorsoDistance(const robot_misc::Human& human, double t) const
{
    if (human.getNumberUncertaintyNeck() > 0)
    {
        Eigen::Matrix<double, -1, -1> distances =
            Eigen::Matrix<double, -1, -1>::Zero(_human_collision_distance_dimension, human.getNumberUncertaintyNeck());
        getHumanUncertaintyThorsoDistances(human, distances, t);
        return distances.minCoeff();
    }
    else
    {
        ROS_ERROR("robot_collision: Called uncertainty distance function without uncertainty");
        return -1;
    }
}

bool RobotCollision::getHumanPelvisDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    return getObstacleDistances(human.getBodyPart("Hip"), d, t);
}

bool RobotCollision::getHumanPelvisDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                                const int uncertainty_instance, double t) const
{
    for (int var = 0; var < human.getNumberUncertaintyLeg(); ++var)
    {
        if (human.getBodyPart("Leg").uncertainty_states.at(static_cast<size_t>(var)).getUncertaintyInstance() == uncertainty_instance)
        {
            // ROS_WARN("Use MS Uncertainty Distance for Pelvis");
            return getUncertaintyObstacleDistances(human.getBodyPart("Leg"), d, t, var);
        }
    }
    return getObstacleDistances(human.getBodyPart("Leg"), d, t, 0);
}

bool RobotCollision::getHumanPelvisDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                                           double t_radius) const
{
    return getObstacleDistances(human.getBodyPart("Hip"), d, t, t_radius);
}

bool RobotCollision::getHumanPelvisDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                             Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                             Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t) const
{
    return getObstacleDistances(human.getBodyPart("Hip"), d, start_points, end_points, t);
}

bool RobotCollision::getHumanUncertaintyPelvisDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d, double t) const
{
    bool success = false;
    for (int var = 0; var < human.getNumberUncertaintyLeg(); ++var)
    {
        success = getUncertaintyObstacleDistances(human.getBodyPart("Leg"), d.col(var), t, var);
    }
    return success;
}

double RobotCollision::getMinHumanPelvisDistance(const robot_misc::Human& human, double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanPelvisDistances(human, distances, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanPelvisDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance, double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanPelvisDistancesUncertaintyInstance(human, distances, uncertainty_instance, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanPelvisDistanceRadiusIncrease(const robot_misc::Human& human, double t, double t_radius) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanPelvisDistancesRadiusIncrease(human, distances, t, t_radius);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanPelvisDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                 Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanPelvisDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanPelvisDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                 Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanPelvisDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    segment     = minRow + 2;
    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanUncertaintyPelvisDistance(const robot_misc::Human& human, double t) const
{
    if (human.getNumberUncertaintyLeg() > 0)
    {
        Eigen::Matrix<double, -1, -1> distances =
            Eigen::Matrix<double, -1, -1>::Zero(_human_collision_distance_dimension, human.getNumberUncertaintyLeg());
        getHumanUncertaintyPelvisDistances(human, distances, t);
        return distances.minCoeff();
    }
    else
    {
        ROS_ERROR("robot_collision: Called uncertainty distance function without uncertainty");
        return -1;
    }
}

bool RobotCollision::getHumanUpperArmRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    return getObstacleDistances(human.getBodyPart("RUArm"), d, t);
}

bool RobotCollision::getHumanUpperArmRightDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                                       const int uncertainty_instance, double t) const
{
    for (int var = 0; var < human.getNumberUncertaintyRUArm(); ++var)
    {
        if (human.getBodyPart("RUArm").uncertainty_states.at(static_cast<size_t>(var)).getUncertaintyInstance() == uncertainty_instance)
        {
            return getUncertaintyObstacleDistances(human.getBodyPart("RUArm"), d, t, var);
        }
    }
    return getObstacleDistances(human.getBodyPart("RUArm"), d, t, 0);
}

bool RobotCollision::getHumanUpperArmRightDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                                  double t, double t_radius) const
{
    return getObstacleDistances(human.getBodyPart("RUArm"), d, t, t_radius);
}

bool RobotCollision::getHumanUpperArmRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                    Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                                    Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t) const
{
    return getObstacleDistances(human.getBodyPart("RUArm"), d, start_points, end_points, t);
}

bool RobotCollision::getHumanUncertaintyUpperArmRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d,
                                                               double t) const
{
    bool success = false;
    for (int var = 0; var < human.getNumberUncertaintyRUArm(); ++var)
    {
        success = getUncertaintyObstacleDistances(human.getBodyPart("RUArm"), d.col(var), t, var);
    }
    return success;
}

double RobotCollision::getMinHumanUpperArmRightDistance(const robot_misc::Human& human, double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanUpperArmRightDistances(human, distances, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanUpperArmRightDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance,
                                                                           double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanUpperArmRightDistancesUncertaintyInstance(human, distances, uncertainty_instance, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanUpperArmRightDistanceRadiusIncrease(const robot_misc::Human& human, double t, double t_radius) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanUpperArmRightDistancesRadiusIncrease(human, distances, t, t_radius);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanUpperArmRightDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                        Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanUpperArmRightDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanUpperArmRightDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                        Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanUpperArmRightDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    segment     = minRow + 2;
    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanUncertaintyUpperArmRightDistance(const robot_misc::Human& human, double t) const
{
    if (human.getNumberUncertaintyRUArm() > 0)
    {
        Eigen::Matrix<double, -1, -1> distances =
            Eigen::Matrix<double, -1, -1>::Zero(_human_collision_distance_dimension, human.getNumberUncertaintyRUArm());
        getHumanUncertaintyUpperArmRightDistances(human, distances, t);
        //        std::cout << "Uperr right uncerainty distances: " << distances << " amount uncertainty: " << human.getNumberUncertaintyRUArm() <<
        //        std::endl;
        return distances.minCoeff();
    }
    else
    {
        ROS_ERROR("robot_collision: Called uncertainty distance function without uncertainty");
        return -1;
    }
}

bool RobotCollision::getHumanLowerArmRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    return getObstacleDistances(human.getBodyPart("RFArm"), d, t);
}

bool RobotCollision::getHumanLowerArmRightDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                                       const int uncertainty_instance, double t) const
{
    for (int var = 0; var < human.getNumberUncertaintyRFArm(); ++var)
    {
        if (human.getBodyPart("RFArm").uncertainty_states.at(static_cast<size_t>(var)).getUncertaintyInstance() == uncertainty_instance)
        {
            // ROS_WARN("Use MS Uncertainty Distance for RFArm");
            return getUncertaintyObstacleDistances(human.getBodyPart("RFArm"), d, t, var);
        }
    }
    return getObstacleDistances(human.getBodyPart("RFArm"), d, t, 0);
}

bool RobotCollision::getHumanLowerArmRightDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                                  double t, double t_radius) const
{
    return getObstacleDistances(human.getBodyPart("RFArm"), d, t, t_radius);
}

bool RobotCollision::getHumanLowerArmRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                    Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                                    Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t) const
{
    return getObstacleDistances(human.getBodyPart("RFArm"), d, start_points, end_points, t);
}

bool RobotCollision::getHumanUncertaintyLowerArmRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d,
                                                               double t) const
{
    bool success = false;
    for (int var = 0; var < human.getNumberUncertaintyRFArm(); ++var)
    {
        success = getUncertaintyObstacleDistances(human.getBodyPart("RFArm"), d.col(var), t, var);
    }
    return success;
}

double RobotCollision::getMinHumanLowerArmRightDistance(const robot_misc::Human& human, double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanLowerArmRightDistances(human, distances, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanLowerArmRightDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance,
                                                                           double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanLowerArmRightDistancesUncertaintyInstance(human, distances, uncertainty_instance, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanLowerArmRightDistanceRadiusIncrease(const robot_misc::Human& human, double t, double t_radius) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanLowerArmRightDistancesRadiusIncrease(human, distances, t, t_radius);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanLowerArmRightDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                        Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanLowerArmRightDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanLowerArmRightDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                        Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanLowerArmRightDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    segment     = minRow + 2;
    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanUncertaintyLowerArmRightDistance(const robot_misc::Human& human, double t) const
{
    if (human.getNumberUncertaintyRFArm() > 0)
    {
        Eigen::Matrix<double, -1, -1> distances =
            Eigen::Matrix<double, -1, -1>::Zero(_human_collision_distance_dimension, human.getNumberUncertaintyRFArm());
        getHumanUncertaintyLowerArmRightDistances(human, distances, t);
        return distances.minCoeff();
    }
    else
    {
        ROS_ERROR("robot_collision: Called uncertainty distance function without uncertainty");
        return -1;
    }
}

bool RobotCollision::getHumanHandRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    return getObstacleDistances(human.getBodyPart("RHand"), d, t);
}

bool RobotCollision::getHumanHandRightDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                                              double t_radius) const
{
    return getObstacleDistances(human.getBodyPart("RHand"), d, t, t_radius);
}

bool RobotCollision::getHumanHandRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                                Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t) const
{
    return getObstacleDistances(human.getBodyPart("RHand"), d, start_points, end_points, t);
}

bool RobotCollision::getHumanUncertaintyHandRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d,
                                                           double t) const
{
    bool success = false;
    for (int var = 0; var < human.getNumberUncertaintyRHand(); ++var)
    {
        success = getUncertaintyObstacleDistances(human.getBodyPart("RHand"), d.col(var), t, var);
    }
    return success;
}

double RobotCollision::getMinHumanHandRightDistance(const robot_misc::Human& human, double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanHandRightDistances(human, distances, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanHandRightDistanceRadiusIncrease(const robot_misc::Human& human, double t, double t_radius) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanHandRightDistancesRadiusIncrease(human, distances, t, t_radius);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanHandRightDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                    Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanHandRightDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanHandRightDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                    Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanHandRightDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    segment     = minRow + 2;
    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanUncertaintyHandRightDistance(const robot_misc::Human& human, double t) const
{
    if (human.getNumberUncertaintyRHand() > 0)
    {
        Eigen::Matrix<double, -1, -1> distances =
            Eigen::Matrix<double, -1, -1>::Zero(_human_collision_distance_dimension, human.getNumberUncertaintyRHand());
        getHumanUncertaintyHandRightDistances(human, distances, t);
        return distances.minCoeff();
    }
    else
    {
        ROS_ERROR("robot_collision: Called uncertainty distance function without uncertainty");
        return -1;
    }
}

bool RobotCollision::getHumanUpperArmLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    return getObstacleDistances(human.getBodyPart("LUArm"), d, t);
}

bool RobotCollision::getHumanUpperArmLeftDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                                      const int uncertainty_instance, double t) const
{
    for (int var = 0; var < human.getNumberUncertaintyLUArm(); ++var)
    {
        if (human.getBodyPart("LUArm").uncertainty_states.at(static_cast<size_t>(var)).getUncertaintyInstance() == uncertainty_instance)
        {
            return getUncertaintyObstacleDistances(human.getBodyPart("LUArm"), d, t, var);
        }
    }
    return getObstacleDistances(human.getBodyPart("LUArm"), d, t, 0);
}

bool RobotCollision::getHumanUpperArmLeftDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                                                 double t_radius) const
{
    return getObstacleDistances(human.getBodyPart("LUArm"), d, t, t_radius);
}

bool RobotCollision::getHumanUpperArmLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                   Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                                   Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t) const
{
    return getObstacleDistances(human.getBodyPart("LUArm"), d, start_points, end_points, t);
}

bool RobotCollision::getHumanUncertaintyUpperArmLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d,
                                                              double t) const
{
    bool success = false;
    for (int var = 0; var < human.getNumberUncertaintyLUArm(); ++var)
    {
        success = getUncertaintyObstacleDistances(human.getBodyPart("LUArm"), d.col(var), t, var);
    }
    return success;
}

double RobotCollision::getMinHumanUpperArmLeftDistance(const robot_misc::Human& human, double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanUpperArmLeftDistances(human, distances, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanUpperArmLeftDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance,
                                                                          double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanUpperArmLeftDistancesUncertaintyInstance(human, distances, uncertainty_instance, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanUpperArmLeftDistanceRadiusIncrease(const robot_misc::Human& human, double t, double t_radius) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanUpperArmLeftDistancesRadiusIncrease(human, distances, t, t_radius);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanUpperArmLeftDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                       Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanUpperArmLeftDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanUpperArmLeftDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                       Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanUpperArmLeftDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    segment     = minRow + 2;
    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanUncertaintyUpperArmLeftDistance(const robot_misc::Human& human, double t) const
{
    if (human.getNumberUncertaintyLUArm() > 0)
    {
        Eigen::Matrix<double, -1, -1> distances =
            Eigen::Matrix<double, -1, -1>::Zero(_human_collision_distance_dimension, human.getNumberUncertaintyLUArm());
        getHumanUncertaintyUpperArmLeftDistances(human, distances, t);

        return distances.minCoeff();
    }
    else
    {
        ROS_ERROR("robot_collision: Called uncertainty distance function without uncertainty");
        return -1;
    }
}
bool RobotCollision::getHumanLowerArmLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    return getObstacleDistances(human.getBodyPart("LFArm"), d, t);
}

bool RobotCollision::getHumanLowerArmLeftDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                                      const int uncertainty_instance, double t) const
{
    for (int var = 0; var < human.getNumberUncertaintyLFArm(); ++var)
    {
        if (human.getBodyPart("LFArm").uncertainty_states.at(static_cast<size_t>(var)).getUncertaintyInstance() == uncertainty_instance)
        {
            // ROS_WARN("Use MS Uncertainty Distance for LFArm");
            return getUncertaintyObstacleDistances(human.getBodyPart("LFArm"), d, t, var);
        }
    }
    return getObstacleDistances(human.getBodyPart("LFArm"), d, t, 0);
}

bool RobotCollision::getHumanLowerArmLeftDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                                                 double t_radius) const
{
    return getObstacleDistances(human.getBodyPart("LFArm"), d, t, t_radius);
}

bool RobotCollision::getHumanLowerArmLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                   Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                                   Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t) const
{
    return getObstacleDistances(human.getBodyPart("LFArm"), d, start_points, end_points, t);
}

bool RobotCollision::getHumanUncertaintyLowerArmLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d,
                                                              double t) const
{
    bool success = false;
    for (int var = 0; var < human.getNumberUncertaintyLFArm(); ++var)
    {
        success = getUncertaintyObstacleDistances(human.getBodyPart("LFArm"), d.col(var), t, var);
    }
    return success;
}

double RobotCollision::getMinHumanLowerArmLeftDistance(const robot_misc::Human& human, double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanUpperArmRightDistances(human, distances, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanLowerArmLeftDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance,
                                                                          double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanUpperArmRightDistancesUncertaintyInstance(human, distances, uncertainty_instance, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanLowerArmLeftDistanceRadiusIncrease(const robot_misc::Human& human, double t, double t_radius) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanUpperArmRightDistancesRadiusIncrease(human, distances, t, t_radius);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanLowerArmLeftDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                       Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanLowerArmLeftDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanLowerArmLeftDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                       Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanLowerArmLeftDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    segment     = minRow + 2;
    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanUncertaintyLowerArmLeftDistance(const robot_misc::Human& human, double t) const
{
    if (human.getNumberUncertaintyLFArm() > 0)
    {
        Eigen::Matrix<double, -1, -1> distances =
            Eigen::Matrix<double, -1, -1>::Zero(_human_collision_distance_dimension, human.getNumberUncertaintyLFArm());
        getHumanUncertaintyLowerArmLeftDistances(human, distances, t);
        return distances.minCoeff();
    }
    else
    {
        ROS_ERROR("robot_collision: Called uncertainty distance function without uncertainty");
        return -1;
    }
}

bool RobotCollision::getHumanHandLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    return getObstacleDistances(human.getBodyPart("LHand"), d, t);
}

bool RobotCollision::getHumanHandLeftDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                                             double t_radius) const
{
    return getObstacleDistances(human.getBodyPart("LHand"), d, t, t_radius);
}

bool RobotCollision::getHumanHandLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                               Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                               Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t) const
{
    return getObstacleDistances(human.getBodyPart("LHand"), d, start_points, end_points, t);
}

bool RobotCollision::getHumanUncertaintyHandLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d, double t) const
{
    bool success = false;
    for (int var = 0; var < human.getNumberUncertaintyLHand(); ++var)
    {
        success = getUncertaintyObstacleDistances(human.getBodyPart("LHand"), d.col(var), t, var);
    }
    return success;
}

double RobotCollision::getMinHumanHandLeftDistance(const robot_misc::Human& human, double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanHandLeftDistances(human, distances, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanHandLeftDistanceRadiusIncrease(const robot_misc::Human& human, double t, double t_radius) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanHandLeftDistancesRadiusIncrease(human, distances, t, t_radius);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanHandLeftDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                   Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanHandLeftDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanHandLeftDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                   Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanHandLeftDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    segment     = minRow + 2;
    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanUncertaintyHandLeftDistance(const robot_misc::Human& human, double t) const
{
    if (human.getNumberUncertaintyLHand() > 0)
    {
        Eigen::Matrix<double, -1, -1> distances =
            Eigen::Matrix<double, -1, -1>::Zero(_human_collision_distance_dimension, human.getNumberUncertaintyLHand());
        getHumanUncertaintyHandLeftDistances(human, distances, t);
        return distances.minCoeff();
    }
    else
    {
        ROS_ERROR("robot_collision: Called uncertainty distance function without uncertainty");
        return -1;
    }
}

bool RobotCollision::getHumanLegDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t) const
{
    return getObstacleDistances(human.getBodyPart("Leg"), d, t);
}

bool RobotCollision::getHumanLegDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                             const int uncertainty_instance, double t) const
{
    for (int var = 0; var < human.getNumberUncertaintyLeg(); ++var)
    {
        if (human.getBodyPart("Leg").uncertainty_states.at(static_cast<size_t>(var)).getUncertaintyInstance() == uncertainty_instance)
        {
            // ROS_WARN("Use MS Uncertainty Distance for Leg");
            return getUncertaintyObstacleDistances(human.getBodyPart("Leg"), d, t, var);
        }
    }
    return getObstacleDistances(human.getBodyPart("Leg"), d, t, 0);
}

bool RobotCollision::getHumanLegDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t,
                                                        double t_radius) const
{
    return getObstacleDistances(human.getBodyPart("Leg"), d, t, t_radius);
}

bool RobotCollision::getHumanLegDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                          Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                          double t) const
{
    return getObstacleDistances(human.getBodyPart("Leg"), d, start_points, end_points, t);
}

bool RobotCollision::getHumanUncertaintyLegDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d, double t) const
{
    bool success = false;
    for (int var = 0; var < human.getNumberUncertaintyLeg(); ++var)
    {
        success = getUncertaintyObstacleDistances(human.getBodyPart("Leg"), d.col(var), t, var);
    }
    return success;
}

double RobotCollision::getMinHumanLegDistance(const robot_misc::Human& human, double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanLegDistances(human, distances, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanLegDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance, double t) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanLegDistancesUncertaintyInstance(human, distances, uncertainty_instance, t);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanLegDistanceRadiusIncrease(const robot_misc::Human& human, double t, double t_radius) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    getHumanLegDistancesRadiusIncrease(human, distances, t, t_radius);
    return distances.minCoeff();
}

double RobotCollision::getMinHumanLegDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                              Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanLegDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanLegDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                              Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_human_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _human_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getHumanLegDistances(human, distances, start_points, end_points, t);
    double dist = distances.minCoeff(&minRow, &minCol);

    segment     = minRow + 2;
    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinHumanUncertaintyLegDistance(const robot_misc::Human& human, double t) const
{
    if (human.getNumberUncertaintyLeg() > 0)
    {
        Eigen::Matrix<double, -1, -1> distances =
            Eigen::Matrix<double, -1, -1>::Zero(_human_collision_distance_dimension, human.getNumberUncertaintyLeg());
        getHumanUncertaintyLegDistances(human, distances, t);
        return distances.minCoeff();
    }
    else
    {
        ROS_ERROR("robot_collision: Called uncertainty distance function without uncertainty");
        return -1;
    }
}

bool RobotCollision::getGroundCollisionDistances(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d) const
{
    return getPlaneCollisionDistances(_ground_plane, d);
}

bool RobotCollision::getGroundCollisionDistances(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                                 Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points) const
{
    return getPlaneCollisionDistances(_ground_plane, d, start_points, end_points);
}

double RobotCollision::getMinGroundCollisionDistance() const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_plane_collision_distance_dimension, 1);
    getGroundCollisionDistances(distances);
    return distances.minCoeff();
}

double RobotCollision::getMinGroundCollisionDistance(Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                     Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_plane_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _plane_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _plane_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getGroundCollisionDistances(distances, start_points, end_points);
    double dist = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

bool RobotCollision::getRoofCollisionDistances(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d) const
{
    return getPlaneCollisionDistances(_roof_plane, d);
}

bool RobotCollision::getRoofCollisionDistances(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                               Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points) const
{
    return getPlaneCollisionDistances(_roof_plane, d, start_points, end_points);
}

double RobotCollision::getMinRoofCollisionDistance() const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_plane_collision_distance_dimension, 1);
    getRoofCollisionDistances(distances);
    return distances.minCoeff();
}

double RobotCollision::getMinRoofCollisionDistance(Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                   Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_plane_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _plane_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _plane_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getRoofCollisionDistances(distances, start_points, end_points);
    double dist = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

double RobotCollision::getMinPlaneCollisionDistance(const robot_misc::Plane& plane) const
{
    Eigen::Matrix<double, -1, 1> distances = Eigen::Matrix<double, -1, 1>::Zero(_plane_collision_distance_dimension, 1);
    getPlaneCollisionDistances(plane, distances);
    return distances.minCoeff();
}

double RobotCollision::getMinPlaneCollisionDistance(const robot_misc::Plane& plane, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                    Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point) const
{
    Eigen::Matrix<double, -1, 1> distances    = Eigen::Matrix<double, -1, 1>::Zero(_plane_collision_distance_dimension, 1);
    Eigen::Matrix<double, 3, -1> start_points = Eigen::Matrix<double, 3, -1>::Zero(3, _plane_collision_distance_dimension);
    Eigen::Matrix<double, 3, -1> end_points   = Eigen::Matrix<double, 3, -1>::Zero(3, _plane_collision_distance_dimension);
    Eigen::MatrixXd::Index minRow, minCol;

    getPlaneCollisionDistances(plane, distances, start_points, end_points);
    double dist = distances.minCoeff(&minRow, &minCol);

    start_point = start_points.col(minRow);
    end_point   = end_points.col(minRow);

    return dist;
}

void RobotCollision::setJointState(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& q) { _robot_kinematic->setJointState(q); }

double RobotCollision::getPointToPointDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p,
                                               const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1,
                                               Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2)
{
    l1 = p;
    l2 = q;
    return (l2 - l1).norm();
}

double RobotCollision::getPointToLineDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p,
                                              const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
                                              const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1,
                                              Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2)
{
    Eigen::Vector3d v, w;
    v = q2 - q1;
    w = p - q1;

    double c1 = w.dot(v);
    if (c1 <= 0)
    {
        return getPointToPointDistance(p, q1, l1, l2);
    }

    double c2 = v.dot(v);
    if (c2 <= c1)
    {
        return getPointToPointDistance(p, q2, l1, l2);
    }

    l1 = p;
    l2 = q1 + (c1 / c2) * v;

    return (l2 - l1).norm();
}

double RobotCollision::getPointToRectangleDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p1,
                                                   const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
                                                   const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2,
                                                   const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q3,
                                                   Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2)
{
    Eigen::Vector3d diff, u, v;
    diff = q1 - p1;
    u    = q2 - q1;
    v    = q3 - q1;

    double a = u.dot(u);
    double b = u.dot(v);
    double c = v.dot(v);
    double d = u.dot(diff);
    double e = v.dot(diff);
    double f = a * c - b * b;

    double s = (b * e - c * d) / f;
    double t = (b * d - a * e) / f;

    s = std::max(std::min(s, 1.0), 0.0);
    t = std::max(std::min(t, 1.0), 0.0);

    l1 = p1;
    l2 = q1 + s * u + t * v;

    return (l2 - l1).norm();
}

double RobotCollision::getPointToExtrudedRectangleDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p1,
                                                           const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
                                                           const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2,
                                                           const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q3,
                                                           Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2)
{
    //------ Fast method assuming the plane is parallel to world x-y
    double d                 = getPointToRectangleDistance(p1, q1, q2, q3, l1, l2);
    const double almost_zero = 0.0001;

    if (l1(2) < l2(2))
    {
        // lower than the plane

        if (fabs(l1(0) - l2(0)) < almost_zero && fabs(l1(1) - l2(1)) < almost_zero)
        {
            // inside ebox, test dist to 4 faces
            Eigen::Vector3d v, w, n, nz, la, lb, l;
            nz << 0.0, 0.0, -1.0;
            v = q2 - q1;
            w = q3 - q1;
            l = l1 - q1;

            double a  = v(0) * w(1) - v(1) * w(0);
            double a1 = l(0) * w(1) - l(1) * w(0);
            double a2 = l(1) * v(0) - l(0) * v(1);

            double s = a1;
            double t = a2;

            d *= -1;

            if (t > s)
            {
                if (t + s > a)
                {
                    n = (v).cross(nz);
                    n.normalize();

                    double dist = getPointToPlaneDistance(p1, q3, n, la, lb);
                    if (dist > d)
                    {
                        d  = dist;
                        l2 = lb;
                    }
                }
                else
                {
                    // t+s <= a
                    n = (w).cross(nz);
                    n.normalize();

                    double dist = getPointToPlaneDistance(p1, q1, n, la, lb);
                    if (dist > d)
                    {
                        d  = dist;
                        l2 = lb;
                    }
                }
            }
            else
            {
                // t <=s
                if (t + s > a)
                {
                    n = (nz).cross(w);
                    n.normalize();

                    double dist = getPointToPlaneDistance(p1, q2, n, la, lb);
                    if (dist > d)
                    {
                        d  = dist;
                        l2 = lb;
                    }
                }
                else
                {
                    // t+s <= a
                    n = (nz).cross(v);
                    n.normalize();

                    double dist = getPointToPlaneDistance(p1, q1, n, la, lb);
                    if (dist > d)
                    {
                        d  = dist;
                        l2 = lb;
                    }
                }
            }
        }
        else
        {
            // next to ebox
            l2(2) = l1(2);
            d     = (l2 - l1).norm();
        }
    }

    return d;

    //------ General method if the plane is allowd to be not parallel to world x-y. In this case it is faster to copy code from
    // getPointToRectangleDistance rather than calling it.
    /*
    Eigen::Vector3d l, v, w, n;
    l = l1 - l2;
    v = q2 - q1;
    w = q3 - q1;
    n = v.cross(w);
    n.normalize();

    if (l.dot(n) < 0.0)
    {
        // lower than the plane
    }
    */
}

double RobotCollision::getPointToPlaneDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p,
                                               const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q,
                                               const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& n, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1,
                                               Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2)
{
    l1       = p;
    double d = n.dot(l1 - q);
    l2       = l1 - d * n;

    return d;
}

double RobotCollision::getLineToLineDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p1,
                                             const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p2,
                                             const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
                                             const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1,
                                             Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2)
{
    Eigen::Vector3d u, v, w;
    u = p2 - p1;
    v = q2 - q1;
    w = p1 - q1;

    double a, b, c, d, e;
    a = u.dot(u);
    b = u.dot(v);
    c = v.dot(v);
    d = u.dot(w);
    e = v.dot(w);

    double sc, sN, sD, tc, tN, tD;
    double D                 = a * c - b * b;
    const double almost_zero = 0.000001;

    sc = sN = sD = D;
    tc = tN = tD = D;

    if (D < almost_zero)
    {
        sN = 0.0;
        sD = 1.0;
        tN = e;
        tD = c;
    }
    else
    {
        sN = b * e - c * d;
        tN = a * e - b * d;

        if (sN < 0.0)
        {
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD)
        {
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0)
    {
        tN = 0.0;

        if (-d < 0.0)
        {
            sN = 0.0;
        }
        else if (-d > a)
        {
            sN = sD;
        }
        else
        {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD)
    {
        tN = tD;
        if (-d + b < 0.0)
        {
            sN = 0.0;
        }
        else if (-d + b > a)
        {
            sN = sD;
        }
        else
        {
            sN = -d + b;
            sD = a;
        }
    }

    sc = (fabs(sN) < almost_zero ? 0.0 : sN / sD);
    tc = (fabs(tN) < almost_zero ? 0.0 : tN / tD);

    l1 = p1 + sc * u;
    l2 = q1 + tc * v;

    double grad_fix_factor = 0.01;
    double eps             = 0.0;
    double dist            = (l2 - l1).norm();
    if (sc < 1 && sc > 0 && tc < 1 && tc > 0 && dist < almost_zero)
    {
        eps += grad_fix_factor * sc * (1 - sc);
        eps += grad_fix_factor * tc * (1 - tc);
    }

    return dist - eps;
}

double RobotCollision::getLineToRectangleDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p1,
                                                  const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p2,
                                                  const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
                                                  const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2,
                                                  const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q3, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1,
                                                  Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2)
{
    Eigen::Vector3d diff, u, v, w;
    diff = q1 - p1;
    u    = p2 - p1;
    v    = q2 - q1;
    w    = q3 - q1;

    const double almost_zero = 0.0001;
    double nu                = u.norm();
    Eigen::Vector3d un       = u / nu;

    double a00 = v.dot(v);
    double a02 = -v.dot(un);
    double a12 = -w.dot(un);
    double a11 = w.dot(w);
    double b0  = v.dot(diff);
    double b1  = w.dot(diff);
    double b2  = -un.dot(diff);

    double a = a00 - a02 * a02;
    double b = -a02 * a12;
    double c = a11 - a12 * a12;
    double d = b0 - a02 * b2;
    double e = b1 - a12 * b2;
    double f = a * c - b * b;

    double zs = (b * e - c * d);
    double zt = (b * d - a * e);

    double s = (fabs(zs) < almost_zero) ? 0.0 : zs / f;
    double t = (fabs(zt) < almost_zero) ? 0.0 : zt / f;

    if (s >= 1.0 || s <= 0.0 || t >= 1.0 || t <= 0.0)
    {
        Eigen::Matrix2d A;
        Eigen::Vector2d B, x;

        A << a, b, b, c;
        B << d, e;
        double min_Q = std::numeric_limits<double>::max();

        double s0 = std::max(std::min(-d / a, 1.0), 0.0);
        x << s0, 0.0;
        double Q = 0.5 * (x.transpose() * A * x + B.transpose() * x)(0);
        if (Q < min_Q)
        {
            min_Q = Q;
            s     = s0;
            t     = 0.0;
        }

        double s1 = std::max(std::min(-(b + d) / a, 1.0), 0.0);
        x << s1, 1.0;
        Q = 0.5 * (x.transpose() * A * x + B.transpose() * x)(0);
        if (Q < min_Q)
        {
            min_Q = Q;
            s     = s1;
            t     = 1.0;
        }

        double t0 = std::max(std::min(-e / c, 1.0), 0.0);
        x << 0.0, t0;
        Q = 0.5 * (x.transpose() * A * x + B.transpose() * x)(0);
        if (Q < min_Q)
        {
            min_Q = Q;
            s     = 0.0;
            t     = t0;
        }

        double t1 = std::max(std::min(-(b + e) / c, 1.0), 0.0);
        x << 1.0, t1;
        Q = 0.5 * (x.transpose() * A * x + B.transpose() * x)(0);
        if (Q < min_Q)
        {
            min_Q = Q;
            s     = 1.0;
            t     = t1;
        }
    }

    double l = -(a02 * s + a12 * t + b2);

    if (l > nu)
    {
        return getPointToRectangleDistance(p2, q1, q2, q3, l1, l2);
    }
    else if (l < 0.0)
    {
        return getPointToRectangleDistance(p1, q1, q2, q3, l1, l2);
    }
    else
    {
        l1 = p1 + l * un;
        l2 = q1 + s * v + t * w;
        return (l2 - l1).norm();
    }
}

double RobotCollision::getLineToExtrudedRectangleDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p1,
                                                          const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p2,
                                                          const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
                                                          const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2,
                                                          const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q3,
                                                          Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2)
{
    //------ Fast method assuming the plane is parallel to world x-y
    double d                 = getLineToRectangleDistance(p1, p2, q1, q2, q3, l1, l2);
    const double almost_zero = 0.0001;

    if (l1(2) < l2(2))
    {
        // lower than the plane

        if (fabs(l1(0) - l2(0)) < almost_zero && fabs(l1(1) - l2(1)) < almost_zero)
        {
            // inside ebox, test dist to 4 faces
            Eigen::Vector3d v, w, n, nz, la, lb, l;
            nz << 0.0, 0.0, -1.0;
            v = q2 - q1;
            w = q3 - q1;
            l = l1 - q1;

            double a  = v(0) * w(1) - v(1) * w(0);
            double a1 = l(0) * w(1) - l(1) * w(0);
            double a2 = l(1) * v(0) - l(0) * v(1);

            double s = a1;
            double t = a2;

            d *= -1;

            if (t > s)
            {
                if (t + s > a)
                {
                    n = (v).cross(nz);
                    n.normalize();

                    double dist = getPointToPlaneDistance(l1, q3, n, la, lb);
                    if (dist > d)
                    {
                        d  = dist;
                        l2 = lb;
                    }
                }
                else
                {
                    // t+s <= a
                    n = (w).cross(nz);
                    n.normalize();

                    double dist = getPointToPlaneDistance(l1, q1, n, la, lb);
                    if (dist > d)
                    {
                        d  = dist;
                        l2 = lb;
                    }
                }
            }
            else
            {
                // t <=s
                if (t + s > a)
                {
                    n = (nz).cross(w);
                    n.normalize();

                    double dist = getPointToPlaneDistance(l1, q2, n, la, lb);
                    if (dist > d)
                    {
                        d  = dist;
                        l2 = lb;
                    }
                }
                else
                {
                    // t+s <= a
                    n = (nz).cross(v);
                    n.normalize();

                    double dist = getPointToPlaneDistance(l1, q1, n, la, lb);
                    if (dist > d)
                    {
                        d  = dist;
                        l2 = lb;
                    }
                }
            }
        }
        else
        {
            // next to ebox
            l2(2) = l1(2);
            d     = (l2 - l1).norm();
        }
    }

    return d;

    //------ General method if the plane is allowd to be not parallel to world x-y. In this case it is faster to copy code from
    // getPointToRectangleDistance rather than calling it.
    /*
    Eigen::Vector3d l, v, w, n;
    l = l1 - l2;
    v = q2 - q1;
    w = q3 - q1;
    n = v.cross(w);
    n.normalize();

    if (l.dot(n) < 0.0)
    {
        // lower than the plane
    }
    */
}

double RobotCollision::getLineToPlaneDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p1,
                                              const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p2,
                                              const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q,
                                              const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& n, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1,
                                              Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2)
{
    double d1 = n.dot(p1 - q);
    double d2 = n.dot(p2 - q);

    if (d1 < d2)
    {
        l1 = p1;
        l2 = p1 - d1 * n;
        return d1;
    }
    else
    {
        l1 = p2;
        l2 = p2 - d2 * n;
        return d2;
    }
}

// MB
double RobotCollision::getLineToRectangleDistanceMB(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p1,
                                                    const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p2,
                                                    const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
                                                    const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2,
                                                    const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q3,
                                                    Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2)
{
    Eigen::Vector3d v1, v2, n;
    v1 = q2 - q1;
    v2 = q3 - q1;
    n  = v1.cross(v2);
    n.normalize();

    // Find point of segment closest to unbounded plane
    double d1 = n.dot(p1 - q1);
    double d2 = n.dot(p2 - q1);

    Eigen::Vector3d closest;
    bool p1_closest = d1 < d2;
    closest << (p1_closest ? p1 : p2);

    Eigen::Matrix3d m;
    m << v1, v2, n;

    // map closest point of segment into the space spanned by the plane
    Eigen::Vector3d c_m = Eigen::PartialPivLU<Eigen::Matrix3d>(m).solve(closest - q1);

    auto unitRange = [](double value) { return value >= 0 && value <= 1; };

    if (unitRange(c_m(0)) && unitRange(c_m(1)))
    {
        // Closest point is within plane
        if (p1_closest)
        {
            l1 = p1;
            l2 = p1 - d1 * n;
            return fabs(d1);
        }
        else
        {
            l1 = p2;
            l2 = p2 - d2 * n;
            return fabs(d2);
        }
    }
    else if (unitRange(c_m(0)) && c_m(1) < 0)
    {
        // Closest segment is q1 + s*v1
        return getLineToLineDistance(p1, p2, q1, q2, l1, l2);
    }
    else if (unitRange(c_m(0)) && c_m(1) > 1)
    {
        // Closest segment is q1+ s*v1 + v2
        return getLineToLineDistance(p1, p2, q3, q2 + v2, l1, l2);
    }
    else if (c_m(0) < 0 && unitRange(c_m(1)))
    {
        // Closest segment is q1 + s*v2
        return getLineToLineDistance(p1, p2, q1, q3, l1, l2);
    }
    else if (c_m(0) > 1 && unitRange(c_m(1)))
    {
        // Closest segment is q1+ v1 + s*v2
        return getLineToLineDistance(p1, p2, q2, q2 + v2, l1, l2);
    }
    else if (c_m(0) > 1 && c_m(1) > 1)
    {
        // Closest point is q2 + v2
        return getPointToLineDistance(q2 + v2, p1, p2, l2, l1);
    }
    else if (c_m(0) > 1 && c_m(1) < 0)
    {
        // Closest point is q2;
        return getPointToLineDistance(q2, p1, p2, l2, l1);
    }
    else if (c_m(0) < 0 && c_m(1) < 0)
    {
        // Closest point is q1
        return getPointToLineDistance(q1, p1, p2, l2, l1);
    }
    else if (c_m(0) < 0 && c_m(1) > 1)
    {
        // Closest point is q3
        return getPointToLineDistance(q3, p1, p2, l2, l1);
    }
    else
    {
        // this case should not be able to occur
        std::stringstream ss;
        ss << c_m;
        ROS_ERROR("Collision case not covered (line to bounded plane); pm = \n%s", ss.str().c_str());
        return std::numeric_limits<double>::max();
    }
}

}  // namespace robot_collision
}  // namespace mhp_robot
