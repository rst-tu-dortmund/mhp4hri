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

#ifndef MOTION_CLASSIFICATION_PROCESS_H
#define MOTION_CLASSIFICATION_PROCESS_H

#include <mhp_robot/robot_obstacle/obstacle_pipeline/base_process.h>

namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

class MotionClassificationProcess : public BaseProcess
{
 public:
    using Ptr  = std::shared_ptr<MotionClassificationProcess>;
    using UPtr = std::unique_ptr<MotionClassificationProcess>;

    MotionClassificationProcess(const std::string& name, double lin_limit, double ang_limit);

    virtual bool process(std::map<int, robot_misc::Obstacle>& static_obstacles, std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                         std::map<int, robot_misc::Human>& humans, std::map<int, robot_misc::UtilityObject>& utility_objects,
                         std::map<int, robot_misc::Plane>& planes, bool forced = false) override;

 private:
    double _linear_limit  = 0.2;
    double _angular_limit = 0.2;
};

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // MOTION_CLASSIFICATION_PROCESS_H
