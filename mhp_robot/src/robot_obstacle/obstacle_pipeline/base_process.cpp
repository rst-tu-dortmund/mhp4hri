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

#include <mhp_robot/robot_obstacle/obstacle_pipeline/base_process.h>

namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

BaseProcess::BaseProcess(const std::string& name) : _name(name) {}

bool BaseProcess::initialize()
{
    _initialized = true;
    return true;
}

bool BaseProcess::reset()
{
    _active      = true;
    _initialized = false;
    return true;
}

bool BaseProcess::isInitialized() const { return _initialized; }

bool BaseProcess::isActive() const { return _active; }

void BaseProcess::setActive(bool active) { _active = active; }

const std::string& BaseProcess::getName() const { return _name; }

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot
