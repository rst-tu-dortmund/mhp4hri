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

#ifndef LOAD_OBSTACLE_PROCESS_H
#define LOAD_OBSTACLE_PROCESS_H

#include <mhp_robot/robot_obstacle/obstacle_pipeline/base_process.h>

namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

class LoadObstacleProcess : public BaseProcess
{
 public:
    using Ptr  = std::shared_ptr<LoadObstacleProcess>;
    using UPtr = std::unique_ptr<LoadObstacleProcess>;

    LoadObstacleProcess(const std::string& name);

    bool process(std::map<int, robot_misc::Obstacle>& static_obstacles, std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                 std::map<int, robot_misc::Human>& humans, std::map<int, robot_misc::UtilityObject>& utility_objects,
                 std::map<int, robot_misc::Plane>& planes, bool forced = false) override;

    bool initialize() override;
    bool reset() override;

 private:
    std::map<int, robot_misc::Obstacle> _static_obstacles;
    std::map<int, robot_misc::Obstacle> _dynamic_obstacles;
    std::map<int, robot_misc::Human> _humans;
    std::map<int, robot_misc::UtilityObject> _utility_objects;
    std::map<int, robot_misc::Plane> _planes;
};

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // LOAD_OBSTACLE_PROCESS_H
