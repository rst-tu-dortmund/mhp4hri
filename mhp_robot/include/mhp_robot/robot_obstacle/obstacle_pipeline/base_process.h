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

#ifndef BASE_PROCESS_H
#define BASE_PROCESS_H

#include <mhp_robot/robot_misc/human.h>
#include <mhp_robot/robot_misc/obstacle.h>
#include <mhp_robot/robot_misc/plane.h>
#include <mhp_robot/robot_misc/utility_object.h>
#include <map>
#include <memory>

namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

class BaseProcess
{
 public:
    using Ptr  = std::shared_ptr<BaseProcess>;
    using UPtr = std::unique_ptr<BaseProcess>;

    BaseProcess(const std::string& name);

    BaseProcess(const BaseProcess&)            = delete;
    BaseProcess(BaseProcess&&)                 = default;
    BaseProcess& operator=(const BaseProcess&) = delete;
    BaseProcess& operator=(BaseProcess&&)      = default;
    virtual ~BaseProcess() {}

    virtual bool process(std::map<int, robot_misc::Obstacle>& static_obstacles, std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                         std::map<int, robot_misc::Human>& humans, std::map<int, robot_misc::UtilityObject>& utility_objects,
                         std::map<int, robot_misc::Plane>& planes, bool forced = false) = 0;

    virtual bool initialize();
    virtual bool reset();

    bool isInitialized() const;
    bool isActive() const;
    void setActive(bool active = true);
    const std::string& getName() const;

 protected:
    bool _active      = true;
    bool _initialized = false;
    std::string _name = "";
};

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // BASE_PROCESS_H
