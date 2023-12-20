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

#include <mhp_robot/robot_obstacle/obstacle_pipeline/state_estimation_process.h>
#include <ros/io.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

StateEstimationProcess::StateEstimationProcess(const std::string& name) : BaseProcess(name)
{
    _current_angles_publisher = _nh.advertise<std_msgs::Float64MultiArray>("/current_angles", 1000);
    _state_vector_publisher   = _nh.advertise<std_msgs::Float64MultiArray>("/state_vector", 1000);

    if (_stop_estimation_time)
    {
        _estimation_time_publisher = _nh.advertise<std_msgs::Float64>("/state_estimation_time", 1000);
    }
}

bool StateEstimationProcess::process(std::map<int, robot_misc::Obstacle>& static_obstacles, std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                                     std::map<int, robot_misc::Human>& humans, std::map<int, robot_misc::UtilityObject>& utility_objects,
                                     std::map<int, robot_misc::Plane>& planes, bool forced)
{
    if (!_initialized)
    {
        ROS_WARN("StateEstimationProcess: Cannot process in an uninitialized state.");
        return false;
    }

    if (_active || forced)
    {
        // measure current time for time stopping
        if (_stop_estimation_time)
        {
            _begin_time_measurement = std::chrono::high_resolution_clock::now();
        }

        // let all estimators predict the current state
        for (auto const& est : _estimators_task_space)
        {
            if (est.second) est.second->predict(ros::Time::now().toSec());
        }

        std::map<int, state_estimators::BaseStateEstimatorTaskSpace::UPtr>::iterator it;
        for (auto& obs : dynamic_obstacles)
        {
            it = _estimators_task_space.find(obs.second.id);
            if (it != _estimators_task_space.end() && it->second)
            {
                // We found an estimator for obs, pass measurement to it
                it->second->update(obs.second.state._poses[0], ros::Time::now().toSec());

                // Use the estimated and corrected state to update obs
                it->second->getState(obs.second.state._poses[0], obs.second.state._linear_velocities[0], obs.second.state._angular_velocities[0],
                                     obs.second.state._linear_accelerations[0], obs.second.state._angular_accelerations[0]);
            }
        }

        // let all estimators predict the current state
        for (auto const& est : _estimators_joint_space)
        {
            if (est.second) est.second->predict(ros::Time::now().toSec());
        }
        std::map<int, state_estimators::BaseStateEstimatorJointSpace::UPtr>::iterator it_js;
        std::map<int, state_estimators::BaseStateEstimatorTaskSpace::UPtr>::iterator it_ts_foot;

        for (auto& hum : humans)
        {

            it_js = _estimators_joint_space.find(hum.second._id);
            if (it_js != _estimators_joint_space.end() && it_js->second && hum.second._poses_updated)
            {
                // update joint Angles in current Human for current Pose (Pose is automatically set by topic_obstacle_process or tf_obstacle_process)
                if (!hum.second._automatic_simulator)
                {
                    hum.second.initializeBodyLength();
                    hum.second.inverseKinematicTf();
                }
                else
                {
                    hum.second.inverseKinematic();
                }
                // get current Joint Angles from the inverse Kinematic
                Eigen::Vector<double, 11> currentAngles(hum.second._joint_angles.data());

                std_msgs::Float64MultiArray angleMsg;
                angleMsg.data = hum.second._joint_angles;
                _current_angles_publisher.publish(angleMsg);

                // make Update step
                it_js->second->update(currentAngles, ros::Time::now().toSec());

                // get new Angles,velocity, accerleration and save angles in Human
                Eigen::Vector<double, 11> newAngles;

                it_js->second->getState(newAngles, hum.second._joint_velocities, hum.second._joint_accelerations);
                std::vector<double> newAnglesVec(newAngles.data(), newAngles.data() + newAngles.size());
                hum.second._joint_angles = newAnglesVec;

                std::vector<double> tmp = newAnglesVec;
                for (int i = 0; i < hum.second._joint_velocities.size(); ++i)
                {
                    tmp.push_back(hum.second._joint_velocities(i));
                }
                for (int i = 0; i < hum.second._joint_accelerations.size(); ++i)
                {
                    tmp.push_back(hum.second._joint_accelerations(i));
                }
                std_msgs::Float64MultiArray stateMsg;
                stateMsg.data = tmp;
                _state_vector_publisher.publish(stateMsg);
            }

            it_ts_foot = _estimators_task_space.find(hum.second._id);
            if (it_ts_foot != _estimators_task_space.end() && it_ts_foot->second && hum.second._poses_updated)  // For Footprint estimation
            {

                Eigen::Matrix4d pose    = Eigen::Matrix4d::Zero();
                Eigen::Matrix4d newPose = Eigen::Matrix4d::Zero();

                Eigen::Vector3d linVel = Eigen::Vector3d::Zero();
                Eigen::Vector3d linAcc = Eigen::Vector3d::Zero();
                Eigen::Vector3d angVel = Eigen::Vector3d::Zero();
                Eigen::Vector3d angAcc = Eigen::Vector3d::Zero();

                pose(0, 3)             = hum.second._foot_print(0);
                pose(1, 3)             = hum.second._foot_print(1);
                pose.block<3, 3>(0, 0) = robot_misc::Common::rotz(hum.second._foot_print(2)).block<3, 3>(0, 0);
                it_ts_foot->second->update(pose, ros::Time::now().toSec());

                it_ts_foot->second->getState(newPose, linVel, angVel, linAcc, angAcc);

                Eigen::Matrix3d relOr       = _mocap_rot.transpose() * newPose.block<3, 3>(0, 0);
                humans.at(0)._foot_print(0) = newPose(0, 3);
                humans.at(0)._foot_print(1) = newPose(1, 3);
                humans.at(0)._foot_print(2) = asin(newPose(1, 0));
                humans.at(0)._foot_print_velocity << linVel(0), linVel(1), angVel(2);
                humans.at(0)._foot_print_acceleration << linAcc(0), linAcc(1), angAcc(2);
            }
        }

        // Publishing estimation time for each process call
        if (_stop_estimation_time)
        {
            _end_time_measurement = std::chrono::high_resolution_clock::now();
            double elapsed_time   = (std::chrono::duration_cast<std::chrono::nanoseconds>(_end_time_measurement - _begin_time_measurement)).count();
            std_msgs::Float64 time_msg;
            time_msg.data = elapsed_time;
            _estimation_time_publisher.publish(time_msg);
        }
    }
    return true;
}

void StateEstimationProcess::setEstimatorsTaskSpace(std::map<int, state_estimators::BaseStateEstimatorTaskSpace::UPtr>& estimators)
{
    _estimators_task_space = std::move(estimators);
}

void StateEstimationProcess::setEstimatorsJointSpace(std::map<int, state_estimators::BaseStateEstimatorJointSpace::UPtr>& estimators)
{
    _estimators_joint_space = std::move(estimators);
}

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot
