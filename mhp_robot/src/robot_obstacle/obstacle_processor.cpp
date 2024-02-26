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

#include <mhp_robot/robot_obstacle/obstacle_processor.h>

namespace mhp_robot {
namespace robot_obstacle {

ObstacleProcessor::ObstacleProcessor(std::vector<obstacle_pipeline::BaseProcess::UPtr>& pipeline) : _pipeline(std::move(pipeline)) {}

void ObstacleProcessor::processPipeline()
{
    for (const obstacle_pipeline::BaseProcess::UPtr& p : _pipeline)
    {
        if (p)
        {
            if (!p->process(_static_obstacles, _dynamic_obstacles, _humans, _utility_objects, _planes))
            {
                ROS_WARN_THROTTLE(20, "Process %s failed partially or completely", p->getName().c_str());
            }
        }
    }
}

void ObstacleProcessor::processPipeline(const std::map<int, robot_misc::Obstacle>& static_obstacles,
                                        const std::map<int, robot_misc::Obstacle>& dynamic_obstacles, const std::map<int, robot_misc::Human>& humans,
                                        const std::map<int, robot_misc::UtilityObject>& utility_objects,
                                        const std::map<int, robot_misc::Plane>& planes)
{
    _static_obstacles  = static_obstacles;
    _dynamic_obstacles = dynamic_obstacles;
    _humans            = humans;
    _utility_objects   = utility_objects;
    _planes            = planes;

    processPipeline();
}

void ObstacleProcessor::initializePipeline()
{
    for (const obstacle_pipeline::BaseProcess::UPtr& p : _pipeline)
    {
        if (p)
            if (!p->isInitialized())
                if (!p->initialize()) ROS_WARN("Process %s failed to initialize", p->getName().c_str());
    }
}

void ObstacleProcessor::reset()
{
    for (const obstacle_pipeline::BaseProcess::UPtr& p : _pipeline)
    {
        if (p)
            if (!p->reset()) ROS_WARN("Process %s failed to reset", p->getName().c_str());
    }
}

}  // namespace robot_obstacle
}  // namespace mhp_robot
