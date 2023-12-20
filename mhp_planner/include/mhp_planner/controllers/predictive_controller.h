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

#ifndef SRC_CONTROLLERS_INCLUDE_MHP_PLANNER_CONTROLLERS_PREDICTIVE_CONTROLLER_H_
#define SRC_CONTROLLERS_INCLUDE_MHP_PLANNER_CONTROLLERS_PREDICTIVE_CONTROLLER_H_

#include <mhp_planner/controllers/controller_interface.h>
#include <mhp_planner/ocp/structured_ocp/structured_optimal_control_problem.h>

#include <ros/ros.h>
#include <memory>
namespace mhp_planner {

/**
 * @brief Predictive controller
 *
 * @ingroup controllers
 *
 * Implementation of a predictive controller that accepts a generic
 * optimal control problem (OCP) that is solved within each step command.
 *
 * The OCP is invoked repeatedly up to a user-specified number of iterations:
 * refer to numOcpIterations().
 *
 * @see ControllerInterface LqrController PidController
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class PredictiveController : public ControllerInterface
{
 public:
    using Ptr = std::shared_ptr<PredictiveController>;

    PredictiveController(){};

    // implements interface method
    int getControlInputDimension() const override { return _ocp ? _ocp->getControlInputDimension() : 0; }
    // implements interface method
    int getStateDimension() const override { return _ocp ? _ocp->getStateDimension() : 0; }
    // implements interface method
    bool hasPiecewiseConstantControls() const override { return _ocp ? _ocp->isConstantControlAction() : false; }
    // implements interface method
    bool providesFutureControls() const override { return _ocp ? _ocp->providesFutureControls() : false; }
    // implements interface method
    bool providesFutureStates() const override { return _ocp ? _ocp->providesFutureStates() : false; }
    // implements interface method
    double getObjectiveValue() const override { return _ocp->getCurrentObjectiveValue(); }

    // implements interface method
    ControllerInterface::Ptr getInstance() const override { return std::make_shared<PredictiveController>(); }
    static Ptr getInstanceStatic() { return std::make_shared<PredictiveController>(); }

    void setOptimalControlProblem(OptimalControlProblemInterface::Ptr ocp) { _ocp = ocp; }
    OptimalControlProblemInterface::Ptr getOptimalControlProblem() { return _ocp; }

    // implements interface method
    bool initialize(const StateVector& x, ReferenceTrajectoryInterface& expected_xref, ReferenceTrajectoryInterface& expected_uref,
                    const Duration& expected_dt, const Time& t, const int id, ReferenceTrajectoryInterface* sref = nullptr) override;
    bool initialize(const StateVector& x, ReferenceTrajectoryInterface& expected_xref, ReferenceTrajectoryInterface& expected_uref,
                    const Duration& expected_dt, const Time& t, ReferenceTrajectoryInterface* sref = nullptr) override
    {
        return initialize(x, expected_xref, expected_uref, expected_dt, t, {}, sref);
    };

    // implements interface method
    bool step(const StateVector& x, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref, const Duration& dt, const Time& t,
              TimeSeries::Ptr u_sequence, TimeSeries::Ptr x_sequence, ReferenceTrajectoryInterface* sref = nullptr,
              ReferenceTrajectoryInterface* xinit = nullptr, ReferenceTrajectoryInterface* uinit = nullptr, const std::string& ns = "",
              bool reinit = false) override;

    // implements interface method
    double getControlDuration() const override { return _ocp ? _ocp->getFirstDt() : 0.0; }

    // implements interface method
    bool supportsAsynchronousControl() const override { return true; }

    // implements interface method
    void reset() override;

    // implements interface method
    int getControllerId() const override { return _id; }

    const int& getNumOcpIterations() const { return _num_ocp_iterations; }
    void setNumOcpIterations(int ocp_iter) { _num_ocp_iterations = ocp_iter; }

    void setOutputControlSequenceLenght(bool activate) { _output_control_sequence = activate; }
    void setOutputStateSequenceLenght(bool activate) { _output_state_sequence = activate; }

    void setAutoUpdatePreviousControl(bool enable) { _auto_update_prev_control = enable; }
    bool getAutoUpdatePreviousControl() const { return _auto_update_prev_control; }

    ControllerStatistics::Ptr getStatistics() const override
    {
        return std::make_shared<ControllerStatistics>(_statistics);
    }  // TODO(roesmann): make_shared?

    void setWarmStart(bool active) override;

    bool getOCPBounds(Eigen::Ref<Eigen::Vector<double, 6>> x_lb, Eigen::Ref<Eigen::Vector<double, 6>> x_ub, Eigen::Ref<Eigen::Vector<double, 6>> u_lb,
                      Eigen::Ref<Eigen::Vector<double, 6>> u_ub) const
    {
        return _ocp->getBounds(x_lb, x_ub, u_lb, u_ub);
    };

    void fromParameterServer(std::string ns) override;

 protected:
    OptimalControlProblemInterface::Ptr _ocp;
    TimeSeries::Ptr _x_ts;
    TimeSeries::Ptr _u_ts;

    ControllerStatistics _statistics;

    bool _auto_update_prev_control = true;

    int _num_ocp_iterations = 1;

    bool _output_control_sequence = false;
    bool _output_state_sequence   = false;

    bool _initialized = false;

    int _id = 0;
};

FACTORY_REGISTER_CONTROLLER(PredictiveController)

}  // namespace mhp_planner

#endif  // SRC_CONTROLLERS_INCLUDE_MHP_PLANNER_CONTROLLERS_PREDICTIVE_CONTROLLER_H_
