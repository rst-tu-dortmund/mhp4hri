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

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <mhp_robot/robot_misc/bounding_box.h>
#include <mhp_robot/robot_misc/state.h>

namespace mhp_robot {
namespace robot_misc {

struct Obstacle
{
    Obstacle()                           = default;
    Obstacle(const Obstacle&)            = default;
    Obstacle(Obstacle&&)                 = default;
    Obstacle& operator=(const Obstacle&) = default;
    Obstacle& operator=(Obstacle&&)      = default;
    virtual ~Obstacle() {}

    robot_misc::BoundingBox bounding_box;
    robot_misc::State state;
    std::vector<robot_misc::State> uncertainty_states;
    int id                = -1;
    std::string name      = "Obstacle";
    int group_id          = 0;
    bool temporary_static = false;
    bool gazebo           = false;
};

}  // namespace robot_misc
}  // namespace mhp_robot

#endif  // OBSTACLE_H
