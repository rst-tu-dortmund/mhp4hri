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

#ifndef SRC_SYSTEMS_INCLUDE_MHP_PLANNER_SYSTEMS_FILTER_INTERFACE_H_
#define SRC_SYSTEMS_INCLUDE_MHP_PLANNER_SYSTEMS_FILTER_INTERFACE_H_

#include <mhp_planner/core/factory.h>
#include <mhp_planner/core/types.h>

#include <ros/ros.h>
#include <memory>

namespace mhp_planner {

/**
 * @brief Interface class for filters
 *
 * @ingroup systems
 *
 * @remark This interface is provided with factory support (FilterFactory).
 *
 * @see MovingAverageFilter MovingMedianFilter
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class FilterInterface
{
 public:
    using Ptr = std::shared_ptr<FilterInterface>;

    //! Default destructor
    virtual ~FilterInterface() = default;

    //! Return a newly created shared instance of the implemented class
    virtual Ptr getInstance() const = 0;

    virtual double filter(double t, double value) = 0;

    //! Reset all internal caches
    virtual void reset() = 0;

    //! Import from parameter server
    virtual bool fromParameterServer(const std::string& ns) = 0;
};

using FilterFactory = Factory<FilterInterface>;
#define FACTORY_REGISTER_FILTER(type) FACTORY_REGISTER_OBJECT(type, FilterInterface)

}  // namespace mhp_planner
#endif  // SRC_SYSTEMS_INCLUDE_MHP_PLANNER_SYSTEMS_FILTER_INTERFACE_H_
