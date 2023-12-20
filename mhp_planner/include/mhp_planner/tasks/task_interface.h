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
#ifndef SRC_TASKS_INCLUDE_MHP_PLANNER_TASKS_TASK_INTERFACE_H_
#define SRC_TASKS_INCLUDE_MHP_PLANNER_TASKS_TASK_INTERFACE_H_

#include <mhp_planner/core/global.h>
#include <mhp_planner/tasks/environment.h>

#include <memory>
#include <string>

namespace mhp_planner {

/**
 * @brief Interface class for tasks
 *
 * @ingroup tasks
 *
 * Possible task implementations can be the closed-loop control of a plant,
 * open-loop control, benchmarking, ...
 *
 * Usually, tasks are called with an Environment to facilitate the
 * initialization and verification of commonly used control architectures.
 * An Environment contains a plant, observer and controller.
 * But a particular task does not need to rely on an Environment only but can contain
 * more objects or replace objects (e.g. multiple controllers, ...).
 *
 * @remark This interface is provided with factory support (TaskFactory).
 *
 * @see Environment
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class TaskInterface
{
 public:
    using Ptr = std::shared_ptr<TaskInterface>;

    //! Virtuel destructor
    virtual ~TaskInterface() {}
    //! Return a newly created shared instance of the implemented class
    virtual Ptr getInstance() const = 0;

    virtual void performTask(Environment& environment, std::string* msg = nullptr) = 0;

    virtual bool verify(const Environment& environment, std::string* msg = nullptr) const = 0;

    //! Reset task state
    virtual void reset() = 0;

    virtual bool fromParameterServer(const std::string& ns) = 0;
#
};

using TaskFactory = Factory<TaskInterface>;
#define FACTORY_REGISTER_TASK(type) FACTORY_REGISTER_OBJECT(type, TaskInterface)

}  // namespace mhp_planner

#endif  // SRC_TASKS_INCLUDE_MHP_PLANNER_TASKS_TASK_INTERFACE_H_
