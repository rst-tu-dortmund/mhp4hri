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
#ifndef SRC_PLANTS_INCLUDE_MHP_PLANNER_PLANTS_PLANT_INTERFACE_H_
#define SRC_PLANTS_INCLUDE_MHP_PLANNER_PLANTS_PLANT_INTERFACE_H_

#include <mhp_planner/core/factory.h>
#include <mhp_planner/core/time.h>
#include <mhp_planner/core/time_series.h>
#include <mhp_planner/core/types.h>
#include <memory>

namespace mhp_planner {

/**
 * @brief Interface class for plants
 *
 * @ingroup plants
 *
 * This class specifies methods that are required to be implemented by specific
 * plants in order to allow their general utilization in a variety of control tasks.
 *
 * @remark This interface is provided with factory support (PlantFactoryPlantFactory).
 *
 * @see SimulatedPlant
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class PlantInterface
{
 public:
    using Ptr           = std::shared_ptr<PlantInterface>;
    using ControlVector = Eigen::VectorXd;
    using StateVector   = Eigen::VectorXd;
    using OutputVector  = Eigen::VectorXd;

    //! Virtual destructor
    virtual ~PlantInterface() {}

    //! Return a newly created shared instance of the implemented class
    virtual Ptr getInstance() const = 0;

    //! Get access to the associated factory
    static Factory<PlantInterface>& getFactory() { return Factory<PlantInterface>::instance(); }

    //! Return the plant input dimension (u)
    virtual int getInputDimension() const = 0;
    //! Return the plant output dimension (y)
    virtual int getOutputDimension() const = 0;

    virtual bool requiresFutureControls() const = 0;

    virtual bool requiresFutureStates() const = 0;

    //! Initialize plant
    virtual bool initialize() { return true; }

    //! Stop plant (you might probably use this to set the plant into a safe setpoint)
    virtual void stop() {}

    virtual void reset() {}

    virtual bool control(const ControlVector& u, const Duration& dt, const Time& t, const std::string& ns = "");

    virtual bool control(const TimeSeries::ConstPtr& u_sequence, const TimeSeries::ConstPtr& x_sequence, const Duration& dt, const Time& t,
                         const std::string& ns = "") = 0;

    virtual bool output(OutputVector& output, const Time& t, const std::string& ns = "") = 0;

    virtual bool setState(const Eigen::Ref<const Eigen::VectorXd>& state) { return false; }

    virtual void getFutureState(const Time& t, const Eigen::Ref<const Eigen::VectorXd>& x, double duration, Eigen::Ref<Eigen::VectorXd> x_predicted)
    {
    }

    virtual bool fromParameterServer(const std::string& ns) = 0;
};

using PlantFactory = Factory<PlantInterface>;
#define FACTORY_REGISTER_PLANT(type) FACTORY_REGISTER_OBJECT(type, PlantInterface)

}  // namespace mhp_planner
#endif  // SRC_PLANTS_INCLUDE_MHP_PLANNER_PLANTS_PLANT_INTERFACE_H_