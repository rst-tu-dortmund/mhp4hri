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

#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>
namespace mhp_robot {
namespace robot_misc {

enum class BoundingBoxType { NOTYPE, SPHERE, CYLINDER, BOX, EBOX };

class BoundingBox
{
 public:
    BoundingBoxType type = BoundingBoxType::NOTYPE;
    double length_x      = 0.0;
    double length_y      = 0.0;
    double radius        = 0.0;
    bool active          = true;
    bool extruded        = false;

    double _dt = 0.1;

    std::vector<double> future_radius;
    Eigen::Matrix<double, 4, 4> T = Eigen::Matrix<double, 4, 4>::Identity();

    double getRadius(double t = 0) const
    {

        if (t == 0)
        {
            return radius;
        }
        else
        {
            size_t idx = t / _dt - 1;
            return (idx < future_radius.size()) ? future_radius.at(idx) : radius;
        }
    }
};

}  // namespace robot_misc
}  // namespace mhp_robot

#endif  // BOUNDING_BOX_H
