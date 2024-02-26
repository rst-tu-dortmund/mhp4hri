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

#ifndef SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_OPTIMAL_CONTROL_PROBLEM_INTERFACE_H_
#define SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_OPTIMAL_CONTROL_PROBLEM_INTERFACE_H_

#include <mhp_planner/core/factory.h>
#include <mhp_planner/core/reference_trajectory.h>
#include <mhp_planner/core/time.h>
#include <mhp_planner/core/types.h>

#include <mhp_planner/ocp/statistics.h>

#include <memory>

namespace mhp_planner {

class OptimalControlProblemInterface
{
 public:
    using Ptr           = std::shared_ptr<OptimalControlProblemInterface>;
    using UPtr          = std::unique_ptr<OptimalControlProblemInterface>;
    using StateVector   = Eigen::VectorXd;
    using ControlVector = Eigen::VectorXd;

    virtual ~OptimalControlProblemInterface() {}

    virtual Ptr getInstance() const = 0;

    virtual void reset() = 0;

    virtual int getControlInputDimension() const = 0;
    virtual int getStateDimension() const        = 0;
    virtual int getN() const                     = 0;
    virtual int getPerformedIterations() const   = 0;

    virtual bool initialize(const int id) { return true; }

    virtual bool compute(const StateVector& x, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref,
                         ReferenceTrajectoryInterface* sref, const Time& t, bool new_run = true, ReferenceTrajectoryInterface* xinit = nullptr,
                         ReferenceTrajectoryInterface* uinit = nullptr, const std::string& ns = "", bool reinit = false) = 0;

    virtual bool getFirstControlInput(ControlVector& u0) const = 0;
    virtual double getFirstDt() const                          = 0;

    virtual bool isConstantControlAction() const = 0;

    virtual double getCurrentObjectiveValue() { return -1; }

    // TODO(roesmann): someday we should change this to return a time-series, e.g. a std::pair<TS,TS>, because pre-allocating a shared pointer is ugly
    virtual void getTimeSeries(TimeSeries::Ptr x_sequence, TimeSeries::Ptr u_sequence, double t_max = MHP_PLANNER_INF_DBL) = 0;
    // virtual std::pair<TimeSeries::Ptr, TimeSeries::Ptr> getTimeSeries(bool states = true, bool controls = true, double t_max = MHP_PLANNER_INF_DBL)
    // = 0;

    virtual bool providesFutureControls() const = 0;
    virtual bool providesFutureStates() const   = 0;

    virtual void setPreviousControlInput(const Eigen::Ref<const ControlVector>& u_prev, double dt) {}
    virtual void setPreviousControlInputDt(double dt) {}

    virtual OptimalControlProblemStatistics::Ptr getStatistics() const { return {}; }

    virtual bool getBounds(Eigen::Ref<Eigen::Vector<double, 6>> x_lb, Eigen::Ref<Eigen::Vector<double, 6>> x_ub,
                           Eigen::Ref<Eigen::Vector<double, 6>> u_lb, Eigen::Ref<Eigen::Vector<double, 6>> u_ub) const
    {
        return false;
    };

    virtual void fromParameterServer(const std::string& ns) = 0;
};

using OptimalControlProgramFactory = Factory<OptimalControlProblemInterface>;
#define FACTORY_REGISTER_OCP(type) FACTORY_REGISTER_OBJECT(type, OptimalControlProblemInterface)

}  // namespace mhp_planner

#endif  // SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_OPTIMAL_CONTROL_PROBLEM_INTERFACE_H_
