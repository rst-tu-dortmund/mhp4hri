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

#ifndef SRC_CONTROLLERS_INCLUDE_MHP_PLANNER_CONTROLLERS_STATISTICS_H_
#define SRC_CONTROLLERS_INCLUDE_MHP_PLANNER_CONTROLLERS_STATISTICS_H_

#include <mhp_planner/core/time.h>
#include <mhp_planner/core/types.h>

#include <memory>

namespace mhp_planner {

struct ControllerStatistics
{
    using Ptr = std::shared_ptr<ControllerStatistics>;

    Duration step_time;

    void clear() { step_time = Duration(0); }
};

}  // namespace mhp_planner

#endif  // SRC_CONTROLLERS_INCLUDE_MHP_PLANNER_CONTROLLERS_STATISTICS_H_
