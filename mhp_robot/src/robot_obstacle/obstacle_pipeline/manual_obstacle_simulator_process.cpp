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

#include <mhp_robot/robot_misc/common.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/manual_obstacle_simulator_process.h>
#include <ros/io.h>

namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

ManualObstacleSimulatorProcess::ManualObstacleSimulatorProcess(const std::string& name) : BaseProcess(name) {}

bool ManualObstacleSimulatorProcess::process(std::map<int, robot_misc::Obstacle>& static_obstacles,
                                             std::map<int, robot_misc::Obstacle>& dynamic_obstacles, std::map<int, robot_misc::Human>& humans,
                                             std::map<int, robot_misc::UtilityObject>& utility_objects, std::map<int, robot_misc::Plane>& planes,
                                             bool forced)
{
    if (!_initialized)
    {
        ROS_WARN("ManualObstacleSimulatorProcess: Cannot process in an uninitialized state.");
        return false;
    }

    if (_active || forced)
    {
        updateInteractiveMarker(dynamic_obstacles);
        _interactive_marker_server->applyChanges();

        assert(dynamic_obstacles.size() == _dynamic_obstacles.size());
    }

    return true;
}

bool ManualObstacleSimulatorProcess::initialize()
{
    ros::NodeHandle n("~");

    _interactive_marker_server = std::make_unique<interactive_markers::InteractiveMarkerServer>(n.getNamespace() + "/interactive_obstacle_marker");

    _initialized = true;

    return true;
}

void ManualObstacleSimulatorProcess::addInteractiveMarker(const robot_misc::Obstacle& obstacle, interactive_markers::InteractiveMarkerServer& server)
{
    visualization_msgs::InteractiveMarker int_marker;
    visualization_msgs::InteractiveMarkerControl obstacle_control;
    obstacle_control.always_visible = true;

    std::ostringstream s;
    s << obstacle.name << " (" << obstacle.id << ")";

    int_marker.name            = std::to_string(obstacle.id);
    int_marker.header.frame_id = "world";
    int_marker.description     = s.str();

    robot_misc::Common::setMarkerPose(int_marker, obstacle);
    int_marker.scale = 0.5;
    int_marker.controls.push_back(obstacle_control);

    visualization_msgs::InteractiveMarkerControl move_control;

    if (obstacle.bounding_box.type == robot_misc::BoundingBoxType::EBOX)
    {
        robot_misc::Common::createControlMarkerReduced(move_control, int_marker);
    }
    else
    {
        robot_misc::Common::createControlMarker(move_control, int_marker);
    }
    server.insert(int_marker, boost::bind(&ManualObstacleSimulatorProcess::callback, this, _1));
}

void ManualObstacleSimulatorProcess::updateObstaclePose(int id, const geometry_msgs::Pose& pose)
{
    // search for right obstacle in _dynamic_obstalces and update pose
    std::map<int, robot_misc::Obstacle>::iterator it = _dynamic_obstacles.find(id);

    if (it != _dynamic_obstacles.end())
    {
        robot_misc::Common::poseMsgToEigen(it->second.state._poses[0], pose);
    }
}

void ManualObstacleSimulatorProcess::updateInteractiveMarker(std::map<int, robot_misc::Obstacle>& map)
{
    std::map<int, robot_misc::Obstacle>::iterator it;

    // go through new list and add if necessary
    for (auto& obs1 : map)
    {
        robot_misc::Obstacle& obs = obs1.second;

        it = _dynamic_obstacles.find(obs.id);

        if (it != _dynamic_obstacles.end())
        {
            // Only copy the pose as we cannot update more
            obs.state._poses[0] = it->second.state._poses[0];
        }
        else
        {
            // we have not found the id in our list, so we add it
            addInteractiveMarker(obs, *_interactive_marker_server);
            _dynamic_obstacles.insert(std::pair<int, robot_misc::Obstacle>(obs.id, obs));
        }
    }

    // go through old list and remove if necessary
    for (auto& obs1 : _dynamic_obstacles)
    {
        robot_misc::Obstacle& obs = obs1.second;

        it = map.find(obs.id);

        if (it == map.end())
        {
            // we have not found the id in the new list, so we remove it from our list
            _interactive_marker_server->erase(std::to_string(obs.id));
            _dynamic_obstacles.erase(obs.id);
        }
    }
}

void ManualObstacleSimulatorProcess::callback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    // update obstacle pose
    updateObstaclePose(std::stoi(feedback->marker_name), feedback->pose);
}

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot
