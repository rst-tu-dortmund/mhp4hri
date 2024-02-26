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

#ifndef SRC_CORE_INCLUDE_MHP_PLANNER_CORE_GLOBAL_H_
#define SRC_CORE_INCLUDE_MHP_PLANNER_CORE_GLOBAL_H_

namespace mhp_planner {

/**
 * @brief global method to check whether to proceed or cancel the current action
 *
 * Time consuming parts of the program should check whether the current state is ok.
 * If not, they should return immediately. This allows other processes to interrupt
 * resepctively preempt long running processes.
 *
 * E.g. the execution of closed-loop control tasks might be configured for a long time duration,
 * but the user wants to stop by setting setOk(bool ok) to false. The tasks can check for
 * ok() frequently and abort its execution in this case.
 *
 * @return true if everything is still ok, false if an abort of the current action is requested.
 */
bool ok();

void setOk(bool ok);

}  // namespace mhp_planner

#endif  // SRC_CORE_INCLUDE_MHP_PLANNER_CORE_GLOBAL_H_
