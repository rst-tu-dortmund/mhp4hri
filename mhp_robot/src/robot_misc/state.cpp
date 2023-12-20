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

#include <mhp_robot/robot_misc/common.h>
#include <mhp_robot/robot_misc/state.h>

namespace mhp_robot {
namespace robot_misc {

State::State()
{
    _poses                             = {Eigen::Matrix<double, 4, 4>::Identity()};
    _linear_velocities                 = {Eigen::Vector3d::Zero()};
    _angular_velocities                = {Eigen::Vector3d::Zero()};
    _linear_accelerations              = {Eigen::Vector3d::Zero()};
    _angular_accelerations             = {Eigen::Vector3d::Zero()};
    _times                             = {0.0};
    _interpolated_pose                 = Eigen::Matrix<double, 4, 4>::Identity();
    _interpolated_linear_velocity      = Eigen::Vector3d::Zero();
    _interpolated_angular_velocity     = Eigen::Vector3d::Zero();
    _interpolated_linear_acceleration  = Eigen::Vector3d::Zero();
    _interpolated_angular_acceleration = Eigen::Vector3d::Zero();
}

const Eigen::Ref<const Eigen::Vector3d> State::getLinearVelocity(double t, State::InterpolationMethod interpolation_method) const
{
    // t < time_zero means approx. current pose
    if (t < time_zero) return _linear_velocities.front();

    // t > tf ZOH for extrapolation
    if (t > _times.back()) return _linear_velocities.back();

    // 0 < t <= tf
    for (int i = 1; i < (int)_times.size(); ++i)
    {
        if (std::fabs(_times[i] - t) < time_zero)
        {
            return _linear_velocities[i];
        }

        if (_times[i] > t)
        {
            if (interpolation_method == NONE)
            {
                // ZOH
                return _linear_velocities[i - 1];
            }
            else if (interpolation_method == LINEAR)
            {
                Common::interpolateVector(_interpolated_linear_velocity, _linear_velocities[i - 1], _linear_velocities[i],
                                          (t - _times[i - 1]) / (_times[i] - _times[i - 1]));
                return _interpolated_linear_velocity;
            }
        }
    }
    ROS_ERROR("state.cpp: No Linear Velocity available, Return Zeros");
    return Eigen::Vector3d::Zero(3, 1);
}

const Eigen::Ref<const Eigen::Vector3d> State::getAngularVelocity(double t, State::InterpolationMethod interpolation_method) const
{
    // t < time_zero means approx. current pose
    if (t < time_zero) return _angular_velocities.front();

    // t > tf ZOH for extrapolation
    if (t > _times.back()) return _angular_velocities.back();

    // 0 < t <= tf
    for (int i = 1; i < (int)_times.size(); ++i)
    {
        if (std::fabs(_times[i] - t) < time_zero)
        {
            return _angular_velocities[i];
        }

        if (_times[i] > t)
        {
            if (interpolation_method == NONE)
            {
                // ZOH
                return _angular_velocities[i - 1];
            }
            else if (interpolation_method == LINEAR)
            {
                Common::interpolateVector(_interpolated_angular_velocity, _angular_velocities[i - 1], _angular_velocities[i],
                                          (t - _times[i - 1]) / (_times[i] - _times[i - 1]));
                return _interpolated_angular_velocity;
            }
        }
    }
    ROS_ERROR("state.cpp: No angular velocity available, Return Zeros");
    return Eigen::Vector3d::Zero(3, 1);
}

const Eigen::Ref<const Eigen::Vector3d> State::getLinearAcceleration(double t, State::InterpolationMethod interpolation_method) const
{
    // t < time_zero means approx. current pose
    if (t < time_zero) return _linear_accelerations.front();

    // t > tf ZOH for extrapolation
    if (t > _times.back()) return _linear_accelerations.back();

    // 0 < t <= tf
    for (int i = 1; i < (int)_times.size(); ++i)
    {
        if (std::fabs(_times[i] - t) < time_zero)
        {
            return _linear_accelerations[i];
        }

        if (_times[i] > t)
        {
            if (interpolation_method == NONE)
            {
                // ZOH
                return _linear_accelerations[i - 1];
            }
            else if (interpolation_method == LINEAR)
            {
                Common::interpolateVector(_interpolated_linear_acceleration, _linear_accelerations[i - 1], _linear_accelerations[i],
                                          (t - _times[i - 1]) / (_times[i] - _times[i - 1]));
                return _interpolated_linear_acceleration;
            }
        }
    }
    ROS_ERROR("state.cpp: No linear acceleration available, Return Zeros");
    return Eigen::Vector3d::Zero(3, 1);
}

const Eigen::Ref<const Eigen::Vector3d> State::getAngularAcceleration(double t, State::InterpolationMethod interpolation_method) const
{
    // t < time_zero means approx. current pose
    if (t < time_zero) return _angular_accelerations.front();

    // t > tf ZOH for extrapolation
    if (t > _times.back()) return _angular_accelerations.back();

    // 0 < t <= tf
    for (int i = 1; i < (int)_times.size(); ++i)
    {
        if (std::fabs(_times[i] - t) < time_zero)
        {
            return _angular_accelerations[i];
        }

        if (_times[i] > t)
        {
            if (interpolation_method == NONE)
            {
                // ZOH
                return _angular_accelerations[i - 1];
            }
            else if (interpolation_method == LINEAR)
            {
                Common::interpolateVector(_interpolated_angular_acceleration, _angular_accelerations[i - 1], _angular_accelerations[i],
                                          (t - _times[i - 1]) / (_times[i] - _times[i - 1]));
                return _interpolated_angular_acceleration;
            }
        }
    }
    ROS_ERROR("state.cpp: No angular acceleration available, Return Zeros");
    return Eigen::Vector3d::Zero(3, 1);
}

const Eigen::Ref<const Eigen::Matrix4d> State::getPose(double t, State::InterpolationMethod interpolation_method) const
{
    // t < time_zero means approx. current pose
    if (t < time_zero) return _poses.front();

    // t > tf ZOH for extrapolation
    if (t > _times.back()) return _poses.back();

    // 0 < t <= tf
    for (int i = 1; i < (int)_times.size(); ++i)
    {
        if (std::fabs(_times[i] - t) < time_zero)
        {
            return _poses[i];
        }

        if (_times[i] > t)
        {
            if (interpolation_method == NONE)
            {
                // ZOH
                return _poses[i - 1];
            }
            else if (interpolation_method == LINEAR)
            {
                Common::interpolatePose(_interpolated_pose, _poses[i - 1], _poses[i], (t - _times[i - 1]) / (_times[i] - _times[i - 1]));
                return _interpolated_pose;
            }
        }
    }
    ROS_ERROR("state.cpp: No Pose available, Return Zeros");
    return Eigen::Matrix4d::Zero(4, 4);
}

void State::clear()
{
    _poses.clear();
    _angular_velocities.clear();
    _linear_velocities.clear();
    _angular_accelerations.clear();
    _linear_accelerations.clear();
    _times.clear();
}
}  // namespace robot_misc
}  // namespace mhp_robot
