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

#include <mhp_planner/controllers/predictive_controller.h>

#include <mhp_planner/core/console.h>

namespace mhp_planner {

bool PredictiveController::initialize(const StateVector& x, ReferenceTrajectoryInterface& expected_xref, ReferenceTrajectoryInterface& expected_uref,
                                      const Duration& expected_dt, const Time& t, const int id, ReferenceTrajectoryInterface* sref)
{
    if (!_ocp || !_ocp->initialize(id))
    {
        PRINT_ERROR("PredictiveController::initialize(): no ocp specified or ocp initialization failed.");
        return false;
    }
    _id          = id;
    _initialized = true;
    return true;
}

bool PredictiveController::step(const ControllerInterface::StateVector& x, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref,
                                const Duration& dt, const Time& t, TimeSeries::Ptr u_sequence, TimeSeries::Ptr x_sequence,
                                ReferenceTrajectoryInterface* sref, ReferenceTrajectoryInterface* xinit, ReferenceTrajectoryInterface* uinit,
                                const std::string& ns, bool reinit)
{
    if (!_initialized)
    {
        if (!initialize(x, xref, uref, dt, t, 0, sref)) return false;
    }

    ControlVector u;

    if (!_ocp)
    {
        PRINT_ERROR("PredictiveController::step(): no ocp specified.");
        return false;
    };

    bool success = false;

    Time t_pre = Time::now();

    for (int i = 0; i < _num_ocp_iterations; ++i)
    {
        success = _ocp->compute(x, xref, uref, sref, t, i == 0, xinit, uinit, ns, reinit);
        if (!success)
        {
            PRINT_ERROR("PredictiveController::step(): ocp computation failed.");
        }
    };

    _statistics.step_time = Time::now() - t_pre;

    success = success && _ocp->getFirstControlInput(u);

    if (_auto_update_prev_control) _ocp->setPreviousControlInput(u, dt.toSec());  // cache control input for next step call

    _ocp->getTimeSeries(x_sequence, u_sequence);
    x_sequence->setTimeFromStart(t.toSec());
    u_sequence->setTimeFromStart(t.toSec());

    // copy optimal control and state sequence
    _x_ts = std::make_shared<TimeSeries>(*x_sequence);
    _u_ts = std::make_shared<TimeSeries>(*u_sequence);

    return success;
}

void PredictiveController::reset()
{
    if (_ocp) _ocp->reset();
}

void PredictiveController::setWarmStart(bool active)
{
    StructuredOptimalControlProblem::Ptr str_ocp = std::dynamic_pointer_cast<StructuredOptimalControlProblem>(_ocp);
    if (str_ocp)
    {
        str_ocp->getDiscretizationGrid()->setWarmStart(active);
    }
    else
    {
        PRINT_WARNING_NAMED("Only StructuredOptimalControlProblem supports warm starting grids");
    }
}

void PredictiveController::fromParameterServer(std::string ns)
{
    ros::NodeHandle nh;
    nh.getParam(ns + "/num_ocp_iterations", _num_ocp_iterations);
    nh.getParam(ns + "/auto_update_prev_control", _auto_update_prev_control);

    // Get OCP from parameter server
    std::string ocp_type;
    nh.getParam(ns + "/ocp/ocp_type", ocp_type);
    OptimalControlProblemInterface::Ptr ocp = OptimalControlProgramFactory::instance().create(ocp_type);
    if (!ocp)
    {
        PRINT_ERROR("PredictiveController::fromParameterServer(): no ocp specified.");
    }
    else
    {
        ocp->fromParameterServer(ns + "/ocp");
        _ocp = ocp;
    }
}

}  // namespace mhp_planner
