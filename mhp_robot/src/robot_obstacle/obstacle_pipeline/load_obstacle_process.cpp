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

#include <mhp_robot/robot_obstacle/obstacle_list.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/load_obstacle_process.h>

namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

LoadObstacleProcess::LoadObstacleProcess(const std::string& name) : BaseProcess(name) {}

bool LoadObstacleProcess::process(std::map<int, robot_misc::Obstacle>& static_obstacles, std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                                  std::map<int, robot_misc::Human>& humans, std::map<int, robot_misc::UtilityObject>& utility_objects,
                                  std::map<int, robot_misc::Plane>& planes, bool forced)
{
    if (!_initialized)
    {
        ROS_WARN("LoadObstacleProcess: Cannot process in an uninitialized state.");
        return false;
    }

    if (_active || forced)
    {
        static_obstacles  = _static_obstacles;
        dynamic_obstacles = _dynamic_obstacles;
        humans            = _humans;
        utility_objects   = _utility_objects;
        planes            = _planes;
        _active           = false;
    }

    return true;
}

bool LoadObstacleProcess::initialize()
{
    ros::NodeHandle n("~");

    // Load obstacles from parameter server
    XmlRpc::XmlRpcValue static_obstacle_param, dynamic_obstacle_param, human_param, utility_object_param, plane_param;

    n.getParam("/obstacles/static_obstacles", static_obstacle_param);
    n.getParam("/obstacles/dynamic_obstacles", dynamic_obstacle_param);
    n.getParam("/obstacles/humans", human_param);
    n.getParam("/obstacles/utility_objects", utility_object_param);
    n.getParam("/obstacles/planes", plane_param);

    _static_obstacles  = ObstacleList::createObstacleMap(static_obstacle_param);
    _dynamic_obstacles = ObstacleList::createObstacleMap(dynamic_obstacle_param);
    _humans            = ObstacleList::createHumanMap(human_param);
    _utility_objects   = ObstacleList::createUtilityObjectMap(utility_object_param);
    _planes            = ObstacleList::createPlaneMap(plane_param);

    _initialized = true;

    return true;
}

bool LoadObstacleProcess::reset()
{
    _initialized = false;
    _active      = true;

    return initialize();
}

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot
