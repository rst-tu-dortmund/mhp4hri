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

#ifndef JOINT_H
#define JOINT_H

#include <math.h>
#include <array>
#include <string>

namespace mhp_robot {
namespace robot_misc {

struct Joint
{
    std::string name         = "Joint";
    std::array<bool, 3> axis = {{false, false, false}};
    bool fixed               = true;
    double offset            = 0.0;
    double tx                = 0.0;
    double ty                = 0.0;
    double tz                = 0.0;
    double Rx                = 0.0;
    double Ry                = 0.0;
    double Rz                = 0.0;
    double max               = M_PI;
    double min               = -M_PI;
    double effort            = 1000;
    double velocity          = 1000;
};

}  // namespace robot_misc
}  // namespace mhp_robot

#endif  // JOINT_H
