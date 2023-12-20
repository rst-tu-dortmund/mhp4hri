/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2023,
 *  TU Dortmund University, Institute of Control Theory and System Engineering
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
*  Authors: Maximilian Krämer
*  Maintainer(s)/Modifier(s): Heiko Renz
 *********************************************************************/

#include <ur10_planner/plants/ur_robot_corbo.h>

namespace mhp_planner {

PlantInterface::Ptr URRobot::getInstance() const { return std::make_shared<URRobot>(); }

bool URRobot::control(const TimeSeries::ConstPtr& u_sequence, const TimeSeries::ConstPtr& x_sequence, const Duration& dt, const Time& t,
                      const std::string& ns)
{
    if (!_initialized)
    {
        PRINT_ERROR("URRobot: please invoke URRobot::initialize() first.");
        return false;
    }

    if (_ur_manipulator.getJointNumber() != u_sequence->getValueDimension())
    {
        PRINT_ERROR(
            "URRobot: dimension of joint_speed vector does not match "
            "number of joints ("
            << _ur_manipulator.getJointNumber() << ").");
        stop();
        return false;
    }

    // sync times
    _ur_manipulator.sync(t.toSec());

    if (_control_mode == ControlMode::CLOSED_LOOP)
    {
        // merge new sequence with old one
        _states.insert(x_sequence);

        // prune sequence to remove old parts
        _states.prune(t.toSec() - _states.getTimeFromStart());

        // assert(time_from_start >= t)

        // make time stamps relative to t
        std::vector<double> time = _states.getTime();
        double tfs               = _states.getTimeFromStart() - t.toSec();
        std::for_each(time.begin(), time.end(), [tfs](double& time) { time += tfs; });

        // closed-loop
        if (!_ignore_controls) _ur_manipulator.performClosedLoopControl(_states.getValuesMatrixView(), time);
    }
    else
    {
        _controls = *u_sequence;
        _controls.setTimeFromStart(t.toSec());  // we are going to execute the controls immedeately

        // open-loop
        if (!_ignore_controls) _ur_manipulator.performOpenLoopControl(_controls.getValuesMatrixView(), _controls.getTime(), _num_controls);
    }

    return true;
}

int URRobot::getInputDimension() const { return _ur_manipulator.getJointNumber(); }

int URRobot::getOutputDimension() const { return _ur_manipulator.getJointNumber(); }

bool URRobot::requiresFutureControls() const { return _num_controls > 1; }

bool URRobot::requiresFutureStates() const { return false; }

bool URRobot::initialize()
{
    if (_initialized) return true;

    if (!_ur_manipulator.initialize(_real_robot, _reset_on_start, std::make_unique<URUtility>(), std::move(_tracker), _feedforward))
    {
        PRINT_ERROR("URRobot: Initializing URManipulator failed.");
        return false;
    }

    _initialized = true;
    PRINT_INFO("URRobot initialized.");

    return true;
}

bool URRobot::output(PlantInterface::OutputVector& output, const Time& t, const std::string& ns)
{
    if (!_initialized)
    {
        PRINT_ERROR("URRobot: please invoke initialize() first.");
        return false;
    }

    if (output.size() != getOutputDimension())
    {
        PRINT_ERROR("URRobot: dimension mismatch.");
        return false;
    }

    std::vector<double> q = _ur_manipulator.getJointState();
    output                = Eigen::Map<PlantInterface::OutputVector>(q.data(), q.size());

    return true;
}

void URRobot::stop()
{
    if (!_initialized) return;

    if (_reset_on_start && !_real_robot) _initialized = false;

    _ur_manipulator.stop();
}

void URRobot::getFutureState(const Time& t, const Eigen::Ref<const Eigen::VectorXd>& x, double duration, Eigen::Ref<Eigen::VectorXd> x_predicted)
{
    // start at current state
    x_predicted = x;

    if (duration <= 0.0) return;

    if (_control_mode == ControlMode::CLOSED_LOOP)
    {
        _states.getValuesInterpolate(t.toSec() - _states.getTimeFromStart() + duration, x_predicted, TimeSeries::Interpolation::Linear,
                                     TimeSeries::Extrapolation::ZeroOrderHold);
        return;
    }
    else
    {
        if (_controls.isEmpty()) return;

        double t0     = t.toSec() - _controls.getTimeFromStart();
        double tf     = t0 + duration;
        int points    = std::min(_num_controls, _controls.getTimeDimension());
        int idx_start = 0;
        int idx_end   = points - 1;

        if (tf < 0.0)
        {
            return;
        }
        else if (t0 > _controls.getTime()[points - 1])
        {
            x_predicted += duration * _controls.getValuesMatrixView().col(points - 1);
            return;
        }

        // look for the first stamp >= t0
        for (int i = 0; i < points; ++i)
        {
            if (_controls.getTime()[i] >= t0)
            {
                idx_start = i;
                break;
            }
        }

        // look for the first stamp <= tf
        for (int i = points - 1; i >= 0; --i)
        {
            if (_controls.getTime()[i] <= tf)
            {
                idx_end = i;
                break;
            }
        }

        // left margin if not at the beginning
        if (idx_start > 0 && _controls.getTime()[idx_start] > t0)
        {
            double dt = _controls.getTime()[idx_start] - t0;
            x_predicted += dt * _controls.getValuesMatrixView().col(idx_start - 1);
        }

        // right margin
        if (_controls.getTime()[idx_end] < tf)
        {
            double dt = tf - _controls.getTime()[idx_end];
            x_predicted += dt * _controls.getValuesMatrixView().col(idx_end);
        }

        // full steps in between
        for (int i = idx_start + 1; i <= idx_end; ++i)
        {
            double dt = _controls.getTime()[i] - _controls.getTime()[i - 1];
            x_predicted += dt * _controls.getValuesMatrixView().col(i - 1);
        }
    }
}

bool URRobot::fromParameterServer(const std::string& ns)
{
    ros::NodeHandle nh;

    // Set configuration parameters
    if (!nh.getParam(ns + "/real_robot", _real_robot))
    {
        PRINT_ERROR("URRobot: Parameter _real_robot failed to read!");
        return false;
    }

    if (!_real_robot)
    {
        if (!nh.getParam(ns + "/reset_on_start", _reset_on_start))
        {
            PRINT_ERROR("URRobot: Parameter _reset_on_start failed to read!");
            return false;
        }
    }
    else
    {
        _reset_on_start = false;
        PRINT_WARNING("URRobot: Parameter _reset_on_start is ignored for real robot!");
    }

    _ignore_controls = false;  // Option to ignore controls for only planning with the real robot

    // Set start joint state
    std::vector<double> default_joint_state;
    if (!nh.getParam(ns + "/start_state", default_joint_state))
    {
        PRINT_ERROR("URRobot: Parameter _default_joint_state failed to read!");
        return false;
    }
    _ur_manipulator.setDefaultJointPosition(default_joint_state);

    // Set control mode
    std::string control_mode_type;
    nh.getParam(ns + "/control_mode/control_mode_type", control_mode_type);

    if (control_mode_type == "OPEN_LOOP")
    {
        // open loop (default)
        _control_mode = ControlMode::OPEN_LOOP;
        if (!nh.getParam(ns + "/control_mode/open_loop_mode/num_controls", _num_controls))
        {
            PRINT_ERROR("URRobot: Parameter _num_controls failed to read!");
            return false;
        }
        _tracker = {};
    }
    else if (control_mode_type == "CLOSED_LOOP")
    {

        _control_mode = ControlMode::CLOSED_LOOP;
        std::vector<double> u_max, u_min, weights;
        if (!nh.getParam(ns + "/control_mode/closed_loop_mode/u_max", u_max))
        {
            PRINT_ERROR("URRobot: Parameter _u_max failed to read!");
            return false;
        }
        if (!nh.getParam(ns + "/control_mode/closed_loop_mode/u_min", u_min))
        {
            PRINT_ERROR("URRobot: Parameter _u_min failed to read!");
            return false;
        }
        if (!nh.getParam(ns + "/control_mode/closed_loop_mode/weights", weights))
        {
            PRINT_ERROR("URRobot: Parameter _weights failed to read!");
            return false;
        }
        _u_max   = Eigen::Map<const Eigen::VectorXd>(u_max.data(), u_max.size());
        _u_min   = Eigen::Map<const Eigen::VectorXd>(u_min.data(), u_min.size());
        _weights = Eigen::Map<const Eigen::ArrayXd>(weights.data(), weights.size());

        // Feedforward of mhp controls
        if (!nh.getParam(ns + "/control_mode/closed_loop_mode/feed_forward", _feedforward))
        {
            PRINT_ERROR("URRobot: Parameter _feed_forward failed to read!");
            return false;
        }

        // P Velocity tracker controller
        _tracker = std::make_unique<RobotPControllerJointSpace>();
        _tracker->setLimits(_u_min, _u_max);
        _tracker->setWeights(_weights);
    }
    else
    {
        PRINT_ERROR("URRobot: Parameter _control_mode_type failed to read!");
        return false;
    }
    return true;
}

}  // namespace mhp_planner
