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

#ifndef SRC_CONTROLLERS_INCLUDE_MHP_PLANNER_CONTROLLERS_CONTROLLER_INTERFACE_H_
#define SRC_CONTROLLERS_INCLUDE_MHP_PLANNER_CONTROLLERS_CONTROLLER_INTERFACE_H_

// #include <mhp_planner/systems/output_function_interface.h>
#include <mhp_planner/controllers/statistics.h>
#include <mhp_planner/core/factory.h>
#include <mhp_planner/core/reference_trajectory.h>
#include <mhp_planner/core/time.h>
#include <mhp_planner/core/types.h>

#include <memory>

namespace mhp_planner {

/**
 * @brief Interface class for controllers
 *
 * @ingroup controllers
 *
 * This class specifies methods that are required to be implemented by specific
 * controllers in order to allow their general utilization in a variety of control tasks.
 *
 * @remark This interface is provided with factory support (ControllerFactory).
 *
 * @see PidController LqrController PredictiveController
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class ControllerInterface
{
 public:
    using Ptr           = std::shared_ptr<ControllerInterface>;
    using UPtr          = std::unique_ptr<ControllerInterface>;
    using StateVector   = Eigen::VectorXd;
    using ControlVector = Eigen::VectorXd;

    //! Virtual destructor
    virtual ~ControllerInterface() {}

    //! Return a newly created shared instance of the implemented class
    virtual Ptr getInstance() const = 0;

    //! Get access to the associated factory
    static Factory<ControllerInterface>& getFactory() { return Factory<ControllerInterface>::instance(); }

    //! Return the control input dimension
    virtual int getControlInputDimension() const = 0;

    virtual int getStateDimension() const = 0;

    virtual bool providesFutureControls() const = 0;

    virtual bool providesFutureStates() const = 0;

    virtual double getObjectiveValue() const { return MHP_PLANNER_INF_DBL; }

    virtual bool hasPiecewiseConstantControls() const = 0;

    virtual bool initialize(const StateVector& x, ReferenceTrajectoryInterface& expected_xref, ReferenceTrajectoryInterface& expected_uref,
                            const Duration& expected_dt, const Time& t, const int id, ReferenceTrajectoryInterface* expected_sref = nullptr)
    {
        return true;
    }

    virtual bool initialize(const StateVector& x, ReferenceTrajectoryInterface& expected_xref, ReferenceTrajectoryInterface& expected_uref,
                            const Duration& expected_dt, const Time& t, ReferenceTrajectoryInterface* expected_sref = nullptr)
    {
        return initialize(x, expected_xref, expected_uref, expected_dt, t, {}, expected_sref);
    }

    virtual int getControllerId() const { return 0; }

    virtual bool step(const StateVector& x, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref, const Duration& dt, const Time& t,
                      TimeSeries::Ptr u_sequence, TimeSeries::Ptr x_sequence, ReferenceTrajectoryInterface* sref = nullptr,
                      ReferenceTrajectoryInterface* xinit = nullptr, ReferenceTrajectoryInterface* uinit = nullptr, const std::string& ns = "",
                      bool reinit = false) = 0;

    //! Return the duration for which the control u obtained from step() is valid (useful for asynchronous control)
    virtual double getControlDuration() const { return 0.0; }
    //! Specify whether the controllers step function is independent of dt and getControlDuration() returns a valid value
    virtual bool supportsAsynchronousControl() const { return false; }

    //! Reset internal controller state and caches
    virtual void reset() = 0;

    virtual ControllerStatistics::Ptr getStatistics() const { return {}; }

    virtual void setWarmStart(bool active) = 0;

    //! Import controller from parameter server
    virtual void fromParameterServer(const std::string ns) = 0;
};

using ControllerFactory = Factory<ControllerInterface>;
#define FACTORY_REGISTER_CONTROLLER(type) FACTORY_REGISTER_OBJECT(type, ControllerInterface)

}  // namespace mhp_planner

#endif  // SRC_CONTROLLERS_INCLUDE_MHP_PLANNER_CONTROLLERS_CONTROLLER_INTERFACE_H_
