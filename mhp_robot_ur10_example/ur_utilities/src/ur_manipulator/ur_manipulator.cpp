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

#include <controller_manager_msgs/SwitchController.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <ur_utilities/ur_manipulator/ur_manipulator.h>
#include <chrono>

namespace mhp_robot {
namespace robot_manipulator {

bool URManipulator::initialize(bool real_robot, bool reset_on_start, URUtility::UPtr ur_utility,
                               robot_trajectory_optimization::RobotPControllerJointSpace::UPtr tracker, bool feed_forward)
{
    // get some infor before we move ur_utility to base class
    _pos_ctrl_names = ur_utility->getPositionControllerNames();
    _vel_ctrl_names = ur_utility->getVelocityControllerNames();

    if (!RobotManipulator::initialize(real_robot, reset_on_start, std::move(ur_utility), std::move(tracker), feed_forward)) return false;

    ros::NodeHandle n("~");

    std::string joint_states_topic   = "";
    std::string joint_speed_topic    = "";
    std::string joint_position_topic = "";

    if (_real_robot)
    {
        // hardcoded topic since real robot does not differ in robot ns
        joint_states_topic = "/ur_driver/joint_states";

        // hardcoded topic since real robot does not differ in robot ns
        joint_speed_topic = "/ur_driver/joint_speed";

        // hardcoded topic since real robot does not differ in robot ns
        joint_position_topic = "/ur_driver/vel_based_pos_traj_controller";
    }
    else
    {
        // will be remapped by launch file to ur3/ur10 ns
        joint_states_topic = "/joint_states";

        // will be remapped by launch file to ur3/ur10 ns
        joint_speed_topic = "joint_speed_controls";

        // will be remapped by launch file to ur3/ur10 ns
        joint_position_topic = "joint_position_controls";
    }

    _joint_states_sub = n.subscribe(joint_states_topic, 1, &RobotManipulator::jointStateCallback, dynamic_cast<RobotManipulator*>(this),
                                    ros::TransportHints().tcpNoDelay());
    ROS_INFO_STREAM("URManipulator: listening for joint states on topic " << joint_states_topic << ".");

    _joint_speed_pub = n.advertise<trajectory_msgs::JointTrajectory>(joint_speed_topic, 1);
    ROS_INFO_STREAM("URManipulator: sending joint speeds on topic " << joint_speed_topic << ".");

    _joint_position_pub = n.advertise<trajectory_msgs::JointTrajectory>(joint_position_topic, 1);
    ROS_INFO_STREAM("URManipulator: sending joint positions on topic " << joint_position_topic << ".");

    if (!ros::topic::waitForMessage<sensor_msgs::JointState>(joint_states_topic, ros::Duration(5)))
    {
        ROS_ERROR_STREAM("URManipulator: " << joint_states_topic << " topic not published.");

        _initialized = false;
        return false;
    }

    std::string switch_controller_service = "";
    if (_real_robot)
    {
        // hardcoded topic since real robot does not differ in robot ns
        switch_controller_service = "/ur_driver/controller_manager/switch_controller";
    }
    else
    {
        // will be remapped by launch file to ur3/ur10 ns
        switch_controller_service = "/controller_manager/switch_controller";
    }
    _controller_switch_srv = n.serviceClient<controller_manager_msgs::SwitchController>(switch_controller_service);

    if (!_controller_switch_srv.waitForExistence(ros::Duration(5)))
    {
        ROS_WARN_STREAM("URManipulator: " << switch_controller_service << " service not avaiable.");

        _initialized = false;
        return false;
    }

    // Stop any lingering robot motion
    setVelocityMode();
    stop();

    // reset on start
    if (!_real_robot && _reset_on_start)
    {
        if (_set_gazebo_joints_service.waitForExistence(ros::Duration(5)))
        {
            resetGazeboPlant();
        }
        else
        {
            ROS_WARN("RobotManipulator: Gazebo service not avaiable. Cannot reset robot");
        }
    }

    _initialized = true;
    ROS_INFO("URManipulator initialized.");

    return true;
}

int URManipulator::getJointNumber() const { return 6; }

void URManipulator::stop()
{
    _publish_open_loop   = false;
    _publish_closed_loop = false;

    // stop threads
    if (_publish_open_loop_worker.joinable()) _publish_open_loop_worker.join();
    if (_publish_closed_loop_worker.joinable()) _publish_closed_loop_worker.join();

    // Check mode (position/velocity) and stop accordingly
    if (_mode == Mode::VELOCITY)
    {
        stopJointVelocity();
    }
    else if (_mode == Mode::POSITION)
    {
        stopJointPosition();
    }

    if (_reset_on_start && !_real_robot)
    {
        _initialized       = false;
        _first_joint_state = true;
    }
}

bool URManipulator::setVelocityMode()
{
    if (_mode == Mode::VELOCITY) return true;

    if (!_controller_switch_srv.exists())
    {
        ROS_ERROR(
            "URManipulator: Cannot switch controllers to velocity mode, "
            "since service is not avaiable");
        return false;
    }

    controller_manager_msgs::SwitchController switch_msg;
    if (_real_robot)
    {
        switch_msg.request.stop_controllers = _pos_ctrl_names;
    }
    else
    {
        switch_msg.request.start_controllers = _vel_ctrl_names;
        switch_msg.request.stop_controllers  = _pos_ctrl_names;
    }

    switch_msg.request.strictness = 1;

    if (_controller_switch_srv.call(switch_msg) && switch_msg.response.ok)
    {
        _mode = Mode::VELOCITY;
        ROS_INFO("URManipulator: Switched to velocity control mode.");
        return true;
    }
    else
    {
        ROS_ERROR("URManipulator: Failed to switch to velocity control mode");
        return false;
    }
}

bool URManipulator::setPositionMode()
{
    if (_mode == Mode::POSITION) return true;

    if (!_controller_switch_srv.exists())
    {
        ROS_ERROR(
            "URManipulator: Cannot switch controllers to position mode, "
            "since service is not avaiable");
        return false;
    }

    controller_manager_msgs::SwitchController switch_msg;
    if (_real_robot)
    {
        switch_msg.request.start_controllers = _pos_ctrl_names;
    }
    else
    {
        switch_msg.request.stop_controllers  = _vel_ctrl_names;
        switch_msg.request.start_controllers = _pos_ctrl_names;
    }

    switch_msg.request.strictness = 1;

    if (_controller_switch_srv.call(switch_msg) && switch_msg.response.ok)
    {
        _mode = Mode::POSITION;
        ROS_INFO("URManipulator: Switched to position mode.");
        return true;
    }
    else
    {
        ROS_ERROR("URManipulator: Failed to switch to position mode");
        return false;
    }
}

void URManipulator::sendSingleOpenLoopCommand(const Eigen::Ref<const Eigen::VectorXd>& command)
{
    trajectory_msgs::JointTrajectory jt;
    setUpJointVelocityCommand(jt, command);
    _joint_speed_pub.publish(jt);

    _control_mutex.lock();
    _sent_controls.push_back(std::make_tuple<Eigen::VectorXd, double>(command, getTaskTime()));
    _control_mutex.unlock();
}

void URManipulator::sendMultipleOpenLoopCommands(const Eigen::Ref<const Eigen::MatrixXd>& u, const std::vector<double>& time, int n_points)
{
    std::vector<trajectory_msgs::JointTrajectory> trajectory_msgs;
    std::vector<double> time_vector;

    trajectory_msgs.resize(n_points);
    time_vector.resize(n_points);

    for (int i = 0; i < n_points; ++i)
    {
        setUpJointVelocityCommand(trajectory_msgs[i], u.col(i));

        if (i + 1 >= n_points)
        {
            // Last command copies second last command
            time_vector[i] = time_vector[i - 1];
        }
        else
        {
            time_vector[i] = 1000 * (time[1 + i] - time[i]);  // ms
        }
    }

    // stop closed-loop worker if running
    if (_publish_closed_loop)
    {
        _publish_closed_loop = false;
        if (_publish_closed_loop_worker.joinable()) _publish_closed_loop_worker.join();
    }

    // copy data for worker thread
    _mutex.lock();
    _trajectory_msgs        = trajectory_msgs;
    _time_vector            = time_vector;
    _new_open_loop_sequence = true;
    _mutex.unlock();

    // Start thread
    if (!_publish_open_loop)
    {
        _publish_open_loop        = true;
        _publish_open_loop_worker = std::thread(&URManipulator::publishOpenLoopCommands, this);
    }
}

void URManipulator::sendMultipleClosedLoopCommands(const Eigen::Ref<const Eigen::MatrixXd>& q, const Eigen::Ref<const Eigen::MatrixXd>& u,
                                                   const std::vector<double>& time)
{
    std::vector<double> time_vector;
    time_vector.resize(time.size());

    // create vector of dts
    for (int i = 0; i < time_vector.size(); ++i)
    {
        if (i + 1 == time_vector.size())
        {
            // Last command copies second last command
            time_vector[i] = time_vector[i - 1];
        }
        else
        {
            time_vector[i] = 1000 * (time[1 + i] - time[i]);  // ms
        }
    }

    // stop open-loop worker if running
    if (_publish_open_loop)
    {
        _publish_open_loop = false;
        if (_publish_open_loop_worker.joinable()) _publish_open_loop_worker.join();
    }

    // copy data for worker thread
    _mutex.lock();
    _states                   = q;
    _ff_controls              = u;
    _time_vector              = time_vector;
    _new_closed_loop_sequence = true;
    _delayed_start            = time[0];
    _mutex.unlock();

    // Start thread
    if (!_publish_closed_loop)
    {
        _publish_closed_loop        = true;
        _publish_closed_loop_worker = std::thread(&URManipulator::publishClosedLoopCommands, this);
    }
}

void URManipulator::sendSingleJointPositionCommand(const Eigen::Ref<const Eigen::VectorXd>& command, double time)
{
    trajectory_msgs::JointTrajectory jt;
    jt.joint_names  = _robot_utility->getJointNames();
    jt.header.stamp = ros::Time::now();

    addJointPositionCommand(jt, command, time);

    _joint_position_pub.publish(jt);
}

void URManipulator::sendMultipleJointPositionCommands(const Eigen::Ref<const Eigen::MatrixXd>& q, const std::vector<double>& time, int n_points)
{
    trajectory_msgs::JointTrajectory jt;
    jt.joint_names  = _robot_utility->getJointNames();
    jt.header.stamp = ros::Time::now();

    for (int i = 0; i < n_points; ++i)
    {
        addJointPositionCommand(jt, q.col(i), time[i]);
    }

    _joint_position_pub.publish(jt);
}

void URManipulator::stopJointVelocity()
{
    trajectory_msgs::JointTrajectory jt;
    setUpJointVelocityCommand(jt, Eigen::VectorXd::Zero(getJointNumber()));
    _joint_speed_pub.publish(jt);
}

void URManipulator::stopJointPosition()
{
    // Send empty trajectory to trigger position hold mode
    trajectory_msgs::JointTrajectory jt;
    _joint_position_pub.publish(jt);
}

void URManipulator::setUpJointVelocityCommand(trajectory_msgs::JointTrajectory& msg, const Eigen::Ref<const Eigen::VectorXd>& joint_speed) const
{
    msg.joint_names = _robot_utility->getJointNames();

    msg.points.resize(1);
    msg.points[0].accelerations = _default_acceleration;
    msg.points[0].velocities.resize(joint_speed.size());

    Eigen::VectorXd::Map(&msg.points[0].velocities[0], joint_speed.size()) = joint_speed;

    msg.header.stamp = ros::Time::now();
}

void URManipulator::addJointPositionCommand(trajectory_msgs::JointTrajectory& msg, const Eigen::Ref<const Eigen::VectorXd>& joint_position,
                                            double time) const
{
    trajectory_msgs::JointTrajectoryPoint point;

    point.time_from_start = ros::Duration(time);
    point.positions.resize(joint_position.size());

    Eigen::VectorXd::Map(&point.positions[0], joint_position.size()) = joint_position;

    msg.points.push_back(point);
}

void URManipulator::publishOpenLoopCommands()
{
    std::chrono::steady_clock::time_point start, end;
    int idx     = 0;
    double time = 0.0;

    while (_publish_open_loop && ros::ok())
    {
        start = std::chrono::steady_clock::now();

        _mutex.lock();

        // start from beginning if there is a new sequence
        if (_new_open_loop_sequence)
        {
            idx                     = 0;
            _new_open_loop_sequence = false;
        }

        if (idx < _trajectory_msgs.size())
        {
            _trajectory_msgs[idx].header.stamp = ros::Time::now();

            if (_publish_open_loop)
            {
                _joint_speed_pub.publish(_trajectory_msgs[idx]);

                _control_mutex.lock();
                Eigen::VectorXd u =
                    Eigen::Map<Eigen::VectorXd>(_trajectory_msgs[idx].points[0].velocities.data(), _trajectory_msgs[idx].points[0].velocities.size());

                _sent_controls.push_back(std::make_tuple<Eigen::VectorXd, double>(Eigen::VectorXd(u), getTaskTime()));
                _control_mutex.unlock();
            }
            else
            {
                break;
            }
        }
        else
        {
            idx = _trajectory_msgs.size() - 1;
        }

        time = _time_vector[idx];
        _mutex.unlock();

        end = std::chrono::steady_clock::now();

        if (_publish_open_loop)
        {
            int delay = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            if (delay < time) std::this_thread::sleep_for(std::chrono::milliseconds((int)time - delay));
        }
        else
        {
            break;
        }

        ++idx;
    }
}

void URManipulator::publishClosedLoopCommands()
{
    std::chrono::steady_clock::time_point start, end;
    int idx     = 0;
    double time = 0.0;
    Eigen::VectorXd u(getJointNumber());
    Eigen::VectorXd x0(getJointNumber());

    while (_publish_closed_loop && ros::ok())
    {
        // idle until the sequence starts
        if (_delayed_start > 0)
        {
            ros::Duration(0.001).sleep();
            _delayed_start -= 0.001;

            continue;
        }

        start = std::chrono::steady_clock::now();

        _mutex.lock();

        // start from beginning if there is a new sequence
        if (_new_closed_loop_sequence)
        {
            idx                       = 0;
            _new_closed_loop_sequence = false;
        }

        // zero order hold extrapolation
        if (idx >= _states.cols())
        {
            idx = _states.cols() - 1;
        }

        // get x0
        x0 = getJointStateEigen();

        // calculate control
        _tracker->step(x0, _states.col(idx), _ff_controls.col(idx), u, _feed_forward);

        // create msg
        trajectory_msgs::JointTrajectory msg;
        setUpJointVelocityCommand(msg, u);

        // send
        if (_publish_closed_loop)
        {
            _joint_speed_pub.publish(msg);

            _control_mutex.lock();
            _sent_controls.push_back(std::make_tuple<Eigen::VectorXd, double>(Eigen::VectorXd(u), getTaskTime()));
            _control_mutex.unlock();
        }
        else
        {
            break;
        }

        time = _time_vector[idx];
        _mutex.unlock();

        end = std::chrono::steady_clock::now();

        if (_publish_closed_loop)
        {
            int delay = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            if (delay < time) std::this_thread::sleep_for(std::chrono::milliseconds((int)time - delay));
        }
        else
        {
            break;
        }

        ++idx;
    }
}

}  // namespace robot_manipulator
}  // namespace mhp_robot
