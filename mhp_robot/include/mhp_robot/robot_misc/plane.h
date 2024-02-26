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

#ifndef PLANE_H
#define PLANE_H

#include <Eigen/Dense>

namespace mhp_robot {
namespace robot_misc {

struct Plane
{
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    Eigen::Vector3d n = (Eigen::Vector3d(3) << 0.0, 0.0, 1.0).finished();
    int id            = -1;
    int group_id      = 0;
    std::string name  = "Plane";
    bool gazebo       = false;
};

}  // namespace robot_misc
}  // namespace mhp_robot

#endif  // PLANE_H
