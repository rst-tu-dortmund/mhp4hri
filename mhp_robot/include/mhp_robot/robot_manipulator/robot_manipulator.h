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

#ifndef ROBOT_MANIPULATOR_H
#define ROBOT_MANIPULATOR_H

#include <mhp_robot/robot_misc/robot_utility.h>
#include <mhp_robot/robot_trajectory_optimization/robot_p_controller_joint_space.h>
#include <memory>
#include <mutex>

namespace mhp_robot {
namespace robot_manipulator {

class RobotManipulator
{
 public:
    using Ptr  = std::shared_ptr<RobotManipulator>;
    using UPtr = std::unique_ptr<RobotManipulator>;

    RobotManipulator();

    RobotManipulator(const RobotManipulator&)            = delete;
    RobotManipulator(RobotManipulator&&)                 = delete;
    RobotManipulator& operator=(const RobotManipulator&) = delete;
    RobotManipulator& operator=(RobotManipulator&&)      = delete;
    virtual ~RobotManipulator() {}

    void performOpenLoopControl(const Eigen::Ref<const Eigen::MatrixXd>& u, const std::vector<double>& time, int num_points = 1);

    void performClosedLoopControl(const Eigen::Ref<const Eigen::MatrixXd>& q, const std::vector<double>& time);

    void sendJointPositionCommands(const Eigen::Ref<const Eigen::MatrixXd>& q, const std::vector<double>& time, int num_points = 1);

    virtual bool initialize(bool real_robot, bool reset_on_start, robot_misc::RobotUtility::UPtr robot_utility,
                            robot_trajectory_optimization::RobotPControllerJointSpace::UPtr tracker = nullptr, bool feed_forward = true);

    virtual int getJointNumber() const = 0;
    virtual void stop()                = 0;

    const std::vector<double>& getJointState() const;
    const std::vector<double>& getJointVelocity() const;
    const Eigen::Ref<const Eigen::VectorXd> getJointStateEigen() const;
    const Eigen::Ref<const Eigen::VectorXd> getJointVelocityEigen() const;

    const std::vector<double>& getDefaultJointPosition() const;
    const Eigen::Ref<const Eigen::VectorXd> getDefaultJointPositionEigen() const;

    void setDefaultJointPosition(const std::vector<double>& joint_position);

    std::vector<std::tuple<Eigen::VectorXd, double>>& getSentControls() const;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    bool isInitialized() const;

    void sync(double time);

    mutable std::mutex _control_mutex;

 protected:
    using RobotUtility               = robot_misc::RobotUtility;
    using RobotPControllerJointSpace = robot_trajectory_optimization::RobotPControllerJointSpace;

    enum class Mode { POSITION, VELOCITY, NONE };
    Mode _mode = Mode::NONE;

    std::vector<double> _current_joint_state;
    std::vector<double> _current_joint_velocity;
    std::vector<double> _default_joint_position;
    std::vector<double> _default_acceleration;

    mutable std::vector<std::tuple<Eigen::VectorXd, double>> _sent_controls;

    Eigen::Map<Eigen::VectorXd> _current_joint_state_vector;
    Eigen::Map<Eigen::VectorXd> _current_joint_velocity_vector;
    Eigen::Map<Eigen::VectorXd> _default_joint_position_vector;

    RobotUtility::UPtr _robot_utility;
    RobotPControllerJointSpace::UPtr _tracker;

    bool _initialized       = false;
    bool _real_robot        = false;
    bool _feed_forward      = true;
    bool _reset_on_start    = false;
    bool _first_joint_state = true;

    ros::ServiceClient _set_gazebo_joints_service;
    std::string _gazebo_model = "";

    double getTaskTime() const;

    virtual void resetGazeboPlant();

    virtual bool setPositionMode();
    virtual bool setVelocityMode();

    virtual void sendSingleOpenLoopCommand(const Eigen::Ref<const Eigen::VectorXd>& command)                                             = 0;
    virtual void sendMultipleOpenLoopCommands(const Eigen::Ref<const Eigen::MatrixXd>& u, const std::vector<double>& time, int n_points) = 0;

    virtual void sendMultipleClosedLoopCommands(const Eigen::Ref<const Eigen::MatrixXd>& q, const Eigen::Ref<const Eigen::MatrixXd>& u,
                                                const std::vector<double>& time) = 0;

    virtual void sendSingleJointPositionCommand(const Eigen::Ref<const Eigen::VectorXd>& command, double time)                                = 0;
    virtual void sendMultipleJointPositionCommands(const Eigen::Ref<const Eigen::MatrixXd>& q, const std::vector<double>& time, int n_points) = 0;

 private:
    Eigen::MatrixXd _feedforward_controls;

    std::chrono::steady_clock::time_point _sync_time;

    double _aux_time = 0.0;

    void initEigenMap();
};

}  // namespace robot_manipulator
}  // namespace mhp_robot

#endif  // ROBOT_MANIPULATOR_H
