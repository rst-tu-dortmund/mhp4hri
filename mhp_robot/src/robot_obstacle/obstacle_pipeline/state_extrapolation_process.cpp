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

#include <mhp_robot/MsgRelativeOrientation.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/state_extrapolation_process.h>
#include <ros/io.h>
#include <std_msgs/Float64.h>
#include <Eigen/Geometry>

namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

StateExtrapolationProcess::StateExtrapolationProcess(const std::string& name, double dt, int N, int extrapolationSteps, bool constant_velocity_model)
    : BaseProcess(name), _dt(dt), _N(N)
{
    _future_angle_pub             = _nh.advertise<MsgFutureAngles>("/extrapolated_angles", 1000);
    _all_poses_extrapolation_pub  = _nh.advertise<MsgRelativeOrientation>("/extrapolation_all_poses", 1000);
    _all_extrapolated_foot_prints = _nh.advertise<MsgRelativeOrientation>("/extrapolation_all_foot_prints", 1000);

    if (_stop_extrapolation_time)
    {
        _extrapolation_time_publisher = _nh.advertise<std_msgs::Float64>("/state_extrapolation_time", 1000);
    }

    if (extrapolationSteps > N)
    {
        _extrapolation_steps = N;
        ROS_WARN("Extrapolation Steps higher than prediction horizont. Adapt extrapolation steps");
    }
    else
    {
        _extrapolation_steps = extrapolationSteps;
    }

    // Parameter for constant velocity model
    _const_vel_model = constant_velocity_model;  // default is false to use the constant acceleration model
    ROS_INFO("Using a constant velocity model for the extrapolation: %d", _const_vel_model);
}

bool StateExtrapolationProcess::process(std::map<int, robot_misc::Obstacle>& static_obstacles, std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                                        std::map<int, robot_misc::Human>& humans, std::map<int, robot_misc::UtilityObject>& utility_objects,
                                        std::map<int, robot_misc::Plane>& planes, bool forced)
{
    if (!_initialized)
    {
        ROS_WARN("StateExtrapolationProcess: Cannot process in an uninitialized state.");
        return false;
    }

    if (_active || forced)
    {
        // Get time for time measurement
        if (_stop_extrapolation_time)
        {
            _begin_time_measurement = std::chrono::high_resolution_clock::now();
        }

        // Extrapolate dynamic obstacles
        for (auto& obs : dynamic_obstacles)
        {
            extrapolate(obs.second);
        }

        for (auto& hum : humans)
        {
            //            std::cout << "Start Human Extrapolating \n" << std::endl;
            extrapolateFootPrint(hum.second, _extrapolation_steps);

            extrapolateJointSpace(hum.second, _extrapolation_steps);

            // Outcomment for publishing all extrapolated data
            MsgRelativeOrientation allExtrapolationsPoses;
            std::vector<double> dataVector;
            for (int i = 0; i < hum.second._body_part_count; ++i)
            {

                for (int k = 0; k <= _extrapolation_steps; ++k)
                {
                    Eigen::Matrix4d tmp = hum.second._body_parts.at(_frame_names.at(i)).state._poses[k];
                    std::vector<double> pose(tmp.data(), tmp.data() + tmp.rows() * tmp.cols());
                    dataVector.insert(dataVector.end(), pose.begin(), pose.end());
                }
            }
            allExtrapolationsPoses.data = dataVector;
            _all_poses_extrapolation_pub.publish(allExtrapolationsPoses);

            // Outcomment for publishing all extrapolated foot prints
            MsgRelativeOrientation allFootPrints;
            std::vector<double> dataVectorFoot;
            Eigen::Vector3d tmp = hum.second._foot_print;

            std::vector<double> footPrint(tmp.data(), tmp.data() + tmp.rows() * tmp.cols());
            dataVectorFoot.insert(dataVectorFoot.end(), footPrint.begin(), footPrint.end());
            for (int k = 0; k <= _extrapolation_steps - 1; ++k)
            {
                Eigen::Vector3d tmp = hum.second._future_foot_prints.at(k);

                std::vector<double> footPrint(tmp.data(), tmp.data() + tmp.rows() * tmp.cols());
                dataVectorFoot.insert(dataVectorFoot.end(), footPrint.begin(), footPrint.end());
            }

            allFootPrints.data = dataVectorFoot;
            _all_extrapolated_foot_prints.publish(allFootPrints);
        }
        // Measure current time and publish elapsed time for extrapolation
        if (_stop_extrapolation_time)
        {
            _end_time_measurement = std::chrono::high_resolution_clock::now();
            double elapsed_time   = (std::chrono::duration_cast<std::chrono::nanoseconds>(_end_time_measurement - _begin_time_measurement)).count();
            std_msgs::Float64 time_msg;
            time_msg.data = elapsed_time;
            _extrapolation_time_publisher.publish(time_msg);
        }
    }
    return true;
}  // namespace obstacle_pipeline

void StateExtrapolationProcess::extrapolate(robot_misc::Obstacle& obstacle) const
{
    // Allocate
    obstacle.state._times.resize(_N);
    obstacle.state._poses.resize(_N);
    obstacle.state._linear_velocities.resize(_N);
    obstacle.state._angular_velocities.resize(_N);
    obstacle.state._linear_accelerations.resize(_N);
    obstacle.state._angular_accelerations.resize(_N);

    // Extrapolate
    for (int i = 1; i < _N; ++i)
    {
        // Accelerations
        obstacle.state._linear_accelerations[i]  = obstacle.state._linear_accelerations[0];
        obstacle.state._angular_accelerations[i] = obstacle.state._angular_accelerations[0];

        // Velocities
        obstacle.state._linear_velocities[i]  = obstacle.state._linear_accelerations[0] * i * _dt + obstacle.state._linear_velocities[0];
        obstacle.state._angular_velocities[i] = obstacle.state._angular_accelerations[0] * i * _dt + obstacle.state._angular_velocities[0];

        // Position
        obstacle.state._poses[i].block<3, 1>(0, 3) = obstacle.state._linear_accelerations[0] * 0.5 * i * i * _dt * _dt +
                                                     obstacle.state._linear_velocities[0] * i * _dt + obstacle.state._poses[0].block<3, 1>(0, 3);
        // Orientation

        // calculate euler vector
        Eigen::Vector3d phi = i * _dt * obstacle.state._angular_velocities[0] + 0.5 * i * i * _dt * _dt * obstacle.state._angular_accelerations[0];

        double n = phi.norm();
        if (n - norm_zero > 0)
        {
            phi = phi / n;
        }
        else
        {
            phi << 1, 1, 1;
        }
        Eigen::AngleAxisd aa(n, phi);

        obstacle.state._poses[i].block<3, 3>(0, 0).noalias() = aa.toRotationMatrix() * obstacle.state._poses[0].block<3, 3>(0, 0);

        // Fill time
        obstacle.state._times[i] = i * _dt;
    }
}

void StateExtrapolationProcess::extrapolateFootPrint(robot_misc::Human& human, int extrapolationSteps) const
{
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity     = Eigen::Vector3d::Zero();
    Eigen::Vector3d foot_print   = Eigen::Vector3d::Zero();

    human.initializeBodyLength();

    // Extrapolate
    human._future_foot_prints.clear();

    for (int i = 1; i <= extrapolationSteps; ++i)
    {

        if (_const_vel_model)
        {
            // Velocities
            velocity = human._foot_print_velocity;

            // Position
            foot_print = human._foot_print + i * _dt * velocity;
            human._future_foot_prints.push_back(foot_print);
        }
        else
        {
            // Accelerations
            acceleration = human._foot_print_acceleration;

            // Velocities
            velocity = human._foot_print_velocity + i * _dt * acceleration;

            // Position
            foot_print = human._foot_print + i * _dt * velocity + 0.5 * i * i * _dt * _dt * acceleration;
            human._future_foot_prints.push_back(foot_print);
        }
        human.updateFutureHipPoses(i, _N);
    }
    if (extrapolationSteps == 0)
    {
        for (int i = extrapolationSteps + 1; i <= _N; ++i)
        {
            human._future_foot_prints.push_back(human._foot_print);
            human.updateFutureHipPoses(i, _N);
        }
    }
    else
    {
        for (int i = extrapolationSteps + 1; i <= _N; ++i)
        {
            human._future_foot_prints.push_back(foot_print);
            human.updateFutureHipPoses(i, _N);
        }
    }
}

void StateExtrapolationProcess::extrapolateJointSpace(robot_misc::Human& human, int extrapolationSteps) const
{
    MsgFutureAngles allAngles;
    MsgExtrapolatedAngle oneAngle;
    Eigen::Vector<double, 11> extrapolatedAngles;
    Eigen::Vector<double, 11> extrapolatedVel;
    Eigen::Vector<double, 11> extrapolatedAcc;

    oneAngle.extrapolationStep = 0;
    oneAngle.angles            = human._joint_angles;

    allAngles.length = 30;
    allAngles.vector_angles.push_back(oneAngle);

    human._all_extrapolated_angles.resize(_N);
    for (int i = 1; i <= extrapolationSteps; ++i)
    {
        if (_const_vel_model)
        {
            // Constant Velocity model
            extrapolatedVel = human._joint_velocities;

            // Angles
            Eigen::Vector<double, 11> currentAngles(human._joint_angles.data());
            extrapolatedAngles = extrapolatedVel * i * _dt + currentAngles;
        }
        else
        {
            // Constant Acceleration model
            extrapolatedAcc = human._joint_accelerations;

            // Velocities
            extrapolatedVel = extrapolatedAcc * i * _dt + human._joint_velocities;

            // Angles
            Eigen::Vector<double, 11> currentAngles(human._joint_angles.data());
            extrapolatedAngles = extrapolatedAcc * 0.5 * i * i * _dt * _dt + human._joint_velocities * i * _dt + currentAngles;
        }
        human._all_extrapolated_angles.at(i - 1) = extrapolatedAngles;

        human.checkJointLimits(extrapolatedAngles);

        human.updateFuturePoses(extrapolatedAngles, i, _N);

        oneAngle.extrapolationStep = i;
        std::vector<double> tmp(extrapolatedAngles.data(), extrapolatedAngles.data() + extrapolatedAngles.rows() * extrapolatedAngles.cols());
        oneAngle.angles = tmp;
        allAngles.vector_angles.push_back(oneAngle);
    }
    if (extrapolationSteps == 0)
    {
        for (int i = extrapolationSteps + 1; i <= _N; ++i)
        {
            Eigen::Vector<double, 11> currentAngles(human._joint_angles.data());

            human._all_extrapolated_angles.at(i - 1) = currentAngles;

            human.updateFuturePoses(currentAngles, i, _N);

            oneAngle.extrapolationStep = i;
            std::vector<double> tmp(currentAngles.data(), currentAngles.data() + currentAngles.rows() * currentAngles.cols());
            oneAngle.angles = tmp;
            allAngles.vector_angles.push_back(oneAngle);
        }
        _future_angle_pub.publish(allAngles);
    }
    else
    {
        for (int k = extrapolationSteps + 1; k <= _N; ++k)
        {
            human._all_extrapolated_angles.at(k - 1) = extrapolatedAngles;

            human.updateFuturePoses(extrapolatedAngles, k, _N);

            oneAngle.extrapolationStep = k;
            std::vector<double> tmp(extrapolatedAngles.data(), extrapolatedAngles.data() + extrapolatedAngles.rows() * extrapolatedAngles.cols());
            oneAngle.angles = tmp;
            allAngles.vector_angles.push_back(oneAngle);
        }

        _future_angle_pub.publish(allAngles);
    }
    //    std::cout << "Current Angle: " << *human._joint_angles.data() << "\n Extrapolated: " << std::endl;
    //    for (auto i : human._all_extrapolated_angles)
    //    {
    //        std::cout << i << std::endl;
    //    }
    //    std::cout << "length Extrapolated: " << human._all_extrapolated_angles.size() << std::endl;
}
}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot
