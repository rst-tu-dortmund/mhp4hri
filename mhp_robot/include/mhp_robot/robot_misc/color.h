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

#ifndef COLOR_H
#define COLOR_H

namespace mhp_robot {
namespace robot_misc {

struct Color
{
    Color(double r, double g, double b, double a = 1.0) : r(r), g(g), b(b), a(a) {}
    Color() {}

    double r = 1.0;
    double g = 1.0;
    double b = 1.0;
    double a = 1.0;
};

}  // namespace robot_misc
}  // namespace mhp_robot

#endif  // COLOR_H
