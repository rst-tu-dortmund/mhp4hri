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
#include <mhp_robot/robot_obstacle/obstacle_pipeline/workspace_filter_process.h>
#include <fstream>
#include <iostream>
namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

WorkspaceFilterProcess::WorkspaceFilterProcess(const std::string& name, double radius) : BaseProcess(name), _workspace_radius(radius) {}

bool WorkspaceFilterProcess::process(std::map<int, robot_misc::Obstacle>& static_obstacles, std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                                     std::map<int, robot_misc::Human>& humans, std::map<int, robot_misc::UtilityObject>& utility_objects,
                                     std::map<int, robot_misc::Plane>& planes, bool forced)
{
    if (!_initialized)
    {
        ROS_WARN("WorkspaceObstacleFilter: Cannot process in an uninitialized state.");
        return false;
    }

    if (_active || forced)
    {
        std::map<int, robot_misc::Obstacle>::iterator obs_it;
        std::map<int, robot_misc::Human>::iterator human_it;
        std::map<int, robot_misc::Plane>::iterator plane_it;

        double distance = 0.0;

        // filter statics
        for (obs_it = static_obstacles.begin(); obs_it != static_obstacles.end();)
        {
            distance = 0;
            if (!robot_collision::RobotCollision::getDistanceBetweenObstacles(distance, _workspace_model, obs_it->second))
                ROS_WARN("WorkspaceObstacleFilter: Distance calculation failed.");

            if (distance > 0)
            {
                obs_it = static_obstacles.erase(obs_it);
            }
            else
            {
                ++obs_it;
            }
        }

        // filter dynamics
        for (obs_it = dynamic_obstacles.begin(); obs_it != dynamic_obstacles.end();)
        {
            distance = 0;
            if (!robot_collision::RobotCollision::getDistanceBetweenObstacles(distance, _workspace_model, obs_it->second))
                ROS_WARN("WorkspaceObstacleFilter: Distance calculation failed.");

            if (distance > 0)
            {
                obs_it = dynamic_obstacles.erase(obs_it);
            }
            else
            {
                ++obs_it;
            }
        }

        // filter humans as soon as the shortest distance is outside the workspace
        for (human_it = humans.begin(); human_it != humans.end();)
        {
            distance = 0;
            if (!robot_collision::RobotCollision::getMinDistanceBetweenHumanObstacle(distance, _workspace_model, human_it->second))
                ROS_WARN("WorkspaceObstacleFilter: Distance calculation failed.");

            if (distance > 0)
            {
                human_it = humans.erase(human_it);
            }
            else
            {
                ++human_it;
            }
        }

        // filter planes
        for (plane_it = planes.begin(); plane_it != planes.end();)
        {
            distance = 0;
            if (!robot_collision::RobotCollision::getDistanceBetweenPlaneObstacle(distance, plane_it->second, _workspace_model))
                ROS_WARN("WorkspaceObstacleFilter: Distance calculation failed.");

            if (distance > 0)
            {
                plane_it = planes.erase(plane_it);
            }
            else
            {
                ++plane_it;
            }
        }
    }

    return true;
}

bool WorkspaceFilterProcess::initialize()
{
    _workspace_model.state._poses[0]     = _workspace_pose;
    _workspace_model.bounding_box.radius = _workspace_radius;
    _workspace_model.bounding_box.type   = robot_misc::BoundingBoxType::SPHERE;

    _initialized = true;

    return true;
}

void WorkspaceFilterProcess::setWorkspaceRadius(double radius)
{
    _workspace_radius = radius;
    initialize();
}

void WorkspaceFilterProcess::setWorkspacePose(const Eigen::Ref<const Eigen::Matrix4d>& pose)
{
    _workspace_pose = pose;
    initialize();
}

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot
