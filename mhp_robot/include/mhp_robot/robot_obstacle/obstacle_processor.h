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

#ifndef OBSTACLE_PROCESSOR_H
#define OBSTACLE_PROCESSOR_H

#include <mhp_robot/robot_obstacle/obstacle_list.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/base_process.h>
#include <memory>

namespace mhp_robot {
namespace robot_obstacle {

class ObstacleProcessor
{
 public:
    using Ptr  = std::shared_ptr<ObstacleProcessor>;
    using UPtr = std::unique_ptr<ObstacleProcessor>;

    ObstacleProcessor(std::vector<obstacle_pipeline::BaseProcess::UPtr>& pipeline);

    ObstacleProcessor(const ObstacleProcessor&)            = delete;
    ObstacleProcessor(ObstacleProcessor&&)                 = default;
    ObstacleProcessor& operator=(const ObstacleProcessor&) = delete;
    ObstacleProcessor& operator=(ObstacleProcessor&&)      = default;
    ~ObstacleProcessor()                                   = default;

    void processPipeline();
    void processPipeline(const std::map<int, robot_misc::Obstacle>& static_obstacles, const std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                         const std::map<int, robot_misc::Human>& humans, const std::map<int, robot_misc::UtilityObject>& utility_objects,
                         const std::map<int, robot_misc::Plane>& planes);

    void initializePipeline();
    void reset();

    std::map<int, robot_misc::Obstacle> _static_obstacles;
    std::map<int, robot_misc::Obstacle> _dynamic_obstacles;
    std::map<int, robot_misc::Human> _humans;
    std::map<int, robot_misc::UtilityObject> _utility_objects;
    std::map<int, robot_misc::Plane> _planes;

 protected:
    std::vector<obstacle_pipeline::BaseProcess::UPtr> _pipeline;
};

}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // OBSTACLE_PROCESSOR_H
