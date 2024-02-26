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

#include <gazebo_msgs/SetModelConfiguration.h>
#include <mhp_robot/robot_manipulator/robot_manipulator.h>

namespace mhp_robot {
namespace robot_manipulator {

RobotManipulator::RobotManipulator()
    : _current_joint_state_vector(nullptr, 0), _current_joint_velocity_vector(nullptr, 0), _default_joint_position_vector(nullptr, 0)
{
}

void RobotManipulator::performOpenLoopControl(const Eigen::Ref<const Eigen::MatrixXd>& u, const std::vector<double>& time, int num_points)
{
    if (time.empty())
    {
        return;
    }

    if (!setVelocityMode())
    {
        ROS_ERROR("RobotManipulator: Failed to switch to velocity mode");
        stop();

        return;
    }

    if (u.cols() != time.size())
    {
        ROS_ERROR("RobotManipulator: Time vector does not match controls");
        stop();

        return;
    }

    if (num_points > u.cols())
    {
        ROS_WARN("RobotManipulator: Not enough commands to send.");
        num_points = u.cols();
    }

    if (getJointNumber() != u.rows())
    {
        ROS_ERROR("RobotManipulator: Mismatch in number of joints");
        stop();

        return;
    }

    if (num_points > 1)
    {
        // Multiple commands
        sendMultipleOpenLoopCommands(u, time, num_points);
    }
    else
    {
        // Single command
        sendSingleOpenLoopCommand(u.col(0));
    }
}

void RobotManipulator::performClosedLoopControl(const Eigen::Ref<const Eigen::MatrixXd>& q, const std::vector<double>& time)
{
    if (time.empty())
    {
        return;
    }

    if (!setVelocityMode())
    {
        ROS_ERROR("RobotManipulator: Failed to switch to velocity mode");
        stop();

        return;
    }

    if (q.cols() != time.size())
    {
        ROS_ERROR("RobotManipulator: Time vector does not match states");
        stop();

        return;
    }

    if (getJointNumber() != q.rows())
    {
        ROS_ERROR("RobotManipulator: Mismatch in number of joints");
        stop();

        return;
    }

    if (!_tracker)
    {
        ROS_ERROR("RobotManipulator: No Closed-Loop Tracker was set");
        stop();

        return;
    }

    // prepare feedforward controls
    _feedforward_controls.resize(q.rows(), q.cols());

    if (_feed_forward)
    {
        int N = q.cols();
        for (int i = 0; i < N - 1; ++i)
        {
            _feedforward_controls.col(i) = (q.col(i + 1) - q.col(i)) / (time[i + 1] - time[i]);
        }
        _feedforward_controls.col(N - 1).setZero();  // last control defaults to 0
    }
    else
    {
        _feedforward_controls.setZero(q.rows(), q.cols());
    }

    sendMultipleClosedLoopCommands(q, _feedforward_controls, time);
}

void RobotManipulator::sendJointPositionCommands(const Eigen::Ref<const Eigen::MatrixXd>& q, const std::vector<double>& time, int num_points)
{
    if (!setPositionMode())
    {
        ROS_ERROR("RobotManipulator: Failed to switch to position mode");
        stop();

        return;
    }

    if (q.cols() != time.size())
    {
        ROS_ERROR("RobotManipulator: Time vector does not match controls");
        stop();

        return;
    }

    if (num_points > q.cols())
    {
        ROS_WARN("RobotManipulator: Not enough commands to send.");
        num_points = q.cols();
    }

    if (getJointNumber() != q.rows())
    {
        ROS_ERROR("RobotManipulator: Mismatch in number of joints");
        stop();

        return;
    }

    if (num_points > 1)
    {
        // Multiple commands
        sendMultipleJointPositionCommands(q, time, num_points);
    }
    else if (num_points == 1)
    {
        // Single command
        double t = 1.0;
        if (time[0] < 1.0)
        {
            ROS_WARN("Trajectory time too short (< 1.0s) to reach end point with safe velocity. Using t = 1.0s");
        }
        else
        {
            t = time[0];
        }
        sendSingleJointPositionCommand(q.col(0), t);
    }
    else
    {
        // Not supported
        ROS_ERROR("RobotManipulator: Number of commands not supported.");
        stop();
    }
}

bool RobotManipulator::initialize(bool real_robot, bool reset_on_start, RobotUtility::UPtr robot_utility,
                                  robot_trajectory_optimization::RobotPControllerJointSpace::UPtr tracker, bool feed_forward)
{
    _real_robot     = real_robot;
    _feed_forward   = feed_forward;
    _reset_on_start = reset_on_start;
    _robot_utility  = std::move(robot_utility);
    _tracker        = std::move(tracker);

    _current_joint_velocity = std::vector<double>(getJointNumber(), 0.0);
    _current_joint_state    = std::vector<double>(getJointNumber(), 0.0);
    _sent_controls.reserve(128);

    initEigenMap();

    _default_acceleration = _robot_utility->getJointAccelerationLimits();

    if (!_real_robot)
    {
        ros::NodeHandle n("~");

        n.getParam("/gazebo_model_name", _gazebo_model);
        ROS_INFO_STREAM("RobotManipulator: Using gazebo model:" << _gazebo_model);

        _set_gazebo_joints_service = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    }

    _initialized = true;

    return true;
}

void RobotManipulator::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    ROS_INFO_ONCE("RobotManipulator: joint_states received.");

    if (msg->name.size() != msg->position.size())
    {
        ROS_ERROR("RobotManipulator: joint states name vector dimension does not match position vector dimension");
        return;
    }

    if (_first_joint_state)
    {
        if (!_robot_utility->initJointMapping(msg->name))
        {
            return;
        }
        _first_joint_state = false;
    }
    _robot_utility->parseJointStates(_current_joint_state, msg->position, true);
    _robot_utility->parseJointStates(_current_joint_velocity, msg->velocity);
}

void RobotManipulator::resetGazeboPlant()
{
    if (_real_robot) return;

    if (_default_joint_position.size() != getJointNumber())
    {
        ROS_WARN("RobotManipulator: Default joint state has wrong dimension.");
        return;
    }

    ROS_WARN("RobotManipulator: SetModelConfiguration causes a wrapping bug with ros_control");

    // Using the Gazebo service call to set model configuration is causing problems with "angle_shortest_path" in the joint state controller

    // Causing a wrapping bug with ros_control
    gazebo_msgs::SetModelConfiguration msg;
    msg.request.model_name      = _gazebo_model;
    msg.request.urdf_param_name = "robot_description";
    msg.request.joint_names     = _robot_utility->getJointNames();
    msg.request.joint_positions = _default_joint_position;

    if (_set_gazebo_joints_service.call(msg))
    {
        ros::Duration(1).sleep();
        ROS_INFO("RobotManipulator: Gazebo model joints reseted.");
    }
    else
    {
        ROS_WARN_STREAM("RobotManipulator: Cannot reset gazebo pose: " << msg.response.status_message);
    }
}

void RobotManipulator::initEigenMap()
{
    new (&_current_joint_state_vector) Eigen::Map<Eigen::VectorXd>(_current_joint_state.data(), getJointNumber());
    new (&_current_joint_velocity_vector) Eigen::Map<Eigen::VectorXd>(_current_joint_velocity.data(), getJointNumber());
    new (&_default_joint_position_vector)
        Eigen::Map<Eigen::VectorXd>(_default_joint_position.data(), static_cast<Eigen::Index>(_default_joint_position.size()));
}

bool RobotManipulator::setPositionMode()
{
    if (_mode == Mode::POSITION) return true;
    _mode = Mode::POSITION;

    return true;
}

bool RobotManipulator::setVelocityMode()
{
    if (_mode == Mode::VELOCITY) return true;
    _mode = Mode::VELOCITY;

    return true;
}

const std::vector<double>& RobotManipulator::getJointState() const { return _current_joint_state; }

const std::vector<double>& RobotManipulator::getJointVelocity() const { return _current_joint_velocity; }

const Eigen::Ref<const Eigen::VectorXd> RobotManipulator::getJointStateEigen() const { return _current_joint_state_vector; }

const Eigen::Ref<const Eigen::VectorXd> RobotManipulator::getJointVelocityEigen() const { return _current_joint_velocity_vector; }

const std::vector<double>& RobotManipulator::getDefaultJointPosition() const { return _default_joint_position; }

const Eigen::Ref<const Eigen::VectorXd> RobotManipulator::getDefaultJointPositionEigen() const { return _default_joint_position_vector; }

void RobotManipulator::setDefaultJointPosition(const std::vector<double>& joint_position)
{
    _default_joint_position = joint_position;
    initEigenMap();
}

std::vector<std::tuple<Eigen::VectorXd, double>>& RobotManipulator::getSentControls() const { return _sent_controls; }

bool RobotManipulator::isInitialized() const { return _initialized; }

void RobotManipulator::sync(double time)
{
    _sync_time = std::chrono::steady_clock::now();
    _aux_time  = time;
}

double RobotManipulator::getTaskTime() const
{
    double duration = (double)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - _sync_time).count();
    return _aux_time + (duration / 1000);
}

}  // namespace robot_manipulator
}  // namespace mhp_robot
