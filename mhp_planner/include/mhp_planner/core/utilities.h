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

#ifndef SRC_CORE_INCLUDE_MHP_PLANNER_CORE_UTILITIES_H_
#define SRC_CORE_INCLUDE_MHP_PLANNER_CORE_UTILITIES_H_

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>

namespace mhp_planner {

namespace util {

#if __cplusplus > 201402L
using std::clamp;
#else

template <class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi)
{
    return std::max(lo, std::min(v, hi));
}
#endif

template <class T>
constexpr const bool is_in_bounds(const T& v, const T& lo, const T& hi)
{
    return !(v < lo) && !(v > hi);
}

template <class T, class Iterator>
inline const bool is_in_bounds_all(Iterator first, Iterator last, const T& lo, const T& hi)
{
    for (Iterator it = first; it != last; ++it)
        if (*it < lo || *it > hi) return false;
    return true;
}

template <class T>
inline int sign(T val)
{
    return (T(0) < val) - (val < T(0));
}

template <class ValT, class SignT>
inline ValT sign(ValT a, SignT b)
{
    return (b > SignT(0) ? std::abs(a) : -std::abs(a));
}

//! Compute the l2 norm of a 2d vector [a,b] as sqrt(a*a+b*b)
inline double l2_norm_2d(double a, double b) { return std::sqrt(a * a + b * b); }

#if __cplusplus >= 201300
using std::make_unique;
#else
template <typename T, typename... Args>
inline std::unique_ptr<T> make_unique(Args&&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
#endif

/// @cond 0 // exclude from doxygen
template <typename... A>
struct variadic_temp_equal : std::false_type
{
};

template <typename A1, typename... Aother, typename B1, typename... Bother>
struct variadic_temp_equal<std::tuple<A1, Aother...>, std::tuple<B1, Bother...>>
{
    static const bool value = std::is_same<A1, B1>::value && variadic_temp_equal<std::tuple<Aother...>, std::tuple<Bother...>>::value;
};

template <typename... B>
struct variadic_temp_equal<std::tuple<>, std::tuple<B...>> : std::true_type
{
};
/// @endcond

}  // namespace util

}  // namespace mhp_planner

#endif  // SRC_CORE_INCLUDE_MHP_PLANNER_CORE_UTILITIES_H_
