/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020,
 *  TU Dortmund University, Institute of Control Theory and System Enginnering
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
 *  Authors: Christoph Rösmann
 *  Modifier(s)/Maintainer(s): Maximilian Krämer, Heiko Renz
 *********************************************************************/

#ifndef SRC_CORE_INCLUDE_MHP_PLANNER_CORE_VALUE_COMPARISON_H_
#define SRC_CORE_INCLUDE_MHP_PLANNER_CORE_VALUE_COMPARISON_H_

#include <algorithm>
#include <cmath>
#include <complex>

namespace mhp_planner {

inline bool approx_equal_abs(double a, double b, double epsilon = 1e-6) { return std::abs(a - b) < epsilon; }

// The double/float comparison definitions can be found in "The art of computer programming by Knuth"

inline bool approx_equal(double a, double b, double epsilon = 1e-6) { return std::abs(a - b) <= std::max(std::abs(a), std::abs(b)) * epsilon; }

inline bool approx_equal(const std::complex<double>& a, const std::complex<double>& b, double epsilon = 1e-6)
{
    return approx_equal(a.real(), b.real(), epsilon) && approx_equal(a.imag(), b.imag(), epsilon);
}

inline bool essentially_equal(double a, double b, double epsilon = 1e-6) { return std::abs(a - b) <= std::min(std::abs(a), std::abs(b)) * epsilon; }

inline bool essentially_equal(const std::complex<double>& a, const std::complex<double>& b, double epsilon = 1e-6)
{
    return essentially_equal(a.real(), b.real(), epsilon) && essentially_equal(a.imag(), b.imag(), epsilon);
}

inline bool approx_zero(double val, double epsilon = 1e-10) { return std::abs(val) <= epsilon; }

inline bool definitely_greater_than(double a, double b, double epsilon = 1e-6) { return (a - b) > std::max(std::abs(a), std::abs(b)) * epsilon; }

inline bool definitely_less_than(double a, double b, double epsilon = 1e-6) { return (b - a) > std::min(std::abs(a), std::abs(b)) * epsilon; }


}  // namespace mhp_planner

#endif  // SRC_CORE_INCLUDE_MHP_PLANNER_CORE_VALUE_COMPARISON_H_
