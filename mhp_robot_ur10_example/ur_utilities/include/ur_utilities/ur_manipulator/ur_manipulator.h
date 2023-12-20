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

#ifndef UR_MANIPULATOR_H
#define UR_MANIPULATOR_H

#include <mhp_robot/robot_manipulator/robot_manipulator.h>
#include <ur_utilities/ur_misc/ur_utility.h>
#include <mutex>
#include <thread>

namespace mhp_robot {
namespace robot_manipulator {

class URManipulator : public RobotManipulator
{
 public:
    using Ptr  = std::shared_ptr<URManipulator>;
    using UPtr = std::unique_ptr<URManipulator>;

    URManipulator() = default;

    bool initialize(bool real_robot, bool reset_on_start, robot_misc::URUtility::UPtr ur_utility, RobotPControllerJointSpace::UPtr tracker = nullptr,
                    bool feed_forward = true);

    int getJointNumber() const override;
    void stop() override;

 private:
    using URUtility = robot_misc::URUtility;

    std::vector<std::string> _pos_ctrl_names;
    std::vector<std::string> _vel_ctrl_names;

    ros::ServiceClient _controller_switch_srv;
    ros::Publisher _joint_speed_pub;
    ros::Publisher _joint_position_pub;
    ros::Subscriber _joint_states_sub;

    // open-loop
    bool _new_open_loop_sequence = false;
    bool _publish_open_loop      = false;
    std::vector<trajectory_msgs::JointTrajectory> _trajectory_msgs;
    std::thread _publish_open_loop_worker;

    // closed-loop
    double _delayed_start          = 0.0;
    bool _new_closed_loop_sequence = false;
    bool _publish_closed_loop      = false;
    Eigen::MatrixXd _states;
    Eigen::MatrixXd _ff_controls;
    std::thread _publish_closed_loop_worker;

    std::vector<double> _time_vector;
    std::mutex _mutex;

    bool setVelocityMode() override;
    bool setPositionMode() override;

    void sendSingleOpenLoopCommand(const Eigen::Ref<const Eigen::VectorXd>& command) override;
    void sendMultipleOpenLoopCommands(const Eigen::Ref<const Eigen::MatrixXd>& u, const std::vector<double>& time, int n_points) override;

    void sendMultipleClosedLoopCommands(const Eigen::Ref<const Eigen::MatrixXd>& q, const Eigen::Ref<const Eigen::MatrixXd>& u,
                                        const std::vector<double>& time) override;

    void sendSingleJointPositionCommand(const Eigen::Ref<const Eigen::VectorXd>& command, double time) override;
    void sendMultipleJointPositionCommands(const Eigen::Ref<const Eigen::MatrixXd>& q, const std::vector<double>& time, int n_points) override;

    void stopJointVelocity();
    void stopJointPosition();
    void setUpJointVelocityCommand(trajectory_msgs::JointTrajectory& msg, const Eigen::Ref<const Eigen::VectorXd>& joint_speed) const;
    void addJointPositionCommand(trajectory_msgs::JointTrajectory& msg, const Eigen::Ref<const Eigen::VectorXd>& joint_position,
                                 double time = 0.0) const;

    void publishOpenLoopCommands();
    void publishClosedLoopCommands();
};

}  // namespace robot_manipulator
}  // namespace mhp_robot

#endif  // UR_MANIPULATOR_H
