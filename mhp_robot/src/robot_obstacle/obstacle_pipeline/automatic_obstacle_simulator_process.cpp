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
#include <mhp_robot/robot_misc/common.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/automatic_obstacle_simulator_process.h>
#include <cmath>
#include <vector>
namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

AutomaticObstacleSimulatorProcess::AutomaticObstacleSimulatorProcess(const std::string& name, bool wait_for_service,
                                                                     const std::string prediction_mode)
    : BaseProcess(name)
{
    _wait_for_service       = wait_for_service;
    _prediction_mode        = prediction_mode;
    _rel_orient_publisher   = _nh.advertise<MsgRelativeOrientation>("/rel_orient", 1000);
    _start_saving_publisher = _nh.advertise<std_msgs::Bool>("/start_saving", 1000);
}

bool AutomaticObstacleSimulatorProcess::process(std::map<int, robot_misc::Obstacle>& static_obstacles,
                                                std::map<int, robot_misc::Obstacle>& dynamic_obstacles, std::map<int, robot_misc::Human>& humans,
                                                std::map<int, robot_misc::UtilityObject>& utility_objects, std::map<int, robot_misc::Plane>& planes,
                                                bool forced)
{
    if (!_initialized)
    {
        ROS_WARN("AutomaticObstacleSimulatorProcess: Cannot process in an uninitialized state.");
        return false;
    }

    if (_active || forced)
    {
        std::map<int, robot_misc::ObstacleTrajectory>::iterator obs_it;
        std::map<int, robot_misc::HumanTrajectory>::iterator hum_it;

        if (!_start)
        {
            for (auto& obs : dynamic_obstacles)
            {
                obs_it = _obstacle_trajectories.find(obs.second.id);

                if (obs_it != _obstacle_trajectories.end())
                {
                    // We have a trajectory for that one
                    getPoseFromTime(obs.second.state._poses[0], obs_it->second, 0);
                }
            }

            for (auto& hum : humans)
            {
                hum_it = _human_trajectories.find(hum.second._id);

                if (hum_it != _human_trajectories.end())
                {
                    // We have a trajectory for that one
                    getHumanPosesFromTime(hum.second, hum_it->second, 0);
                    hum.second._automatic_simulator = true;
                    hum.second._poses_updated       = true;
                }
            }

            _first_start = true;
        }

        if (_first_start)
        {
            _start_time  = ros::Time::now();
            _first_start = false;
        }

        for (auto& obs : dynamic_obstacles)
        {
            obs_it = _obstacle_trajectories.find(obs.second.id);

            if (obs_it != _obstacle_trajectories.end())
            {
                // We have a trajectory for that one
                const robot_misc::ObstacleTrajectory& obstacle_trajectory = obs_it->second;

                if (obstacle_trajectory.loop)
                {
                    getPoseFromTime(obs.second.state._poses[0], obstacle_trajectory,
                                    std::fmod((ros::Time::now() - _start_time).toSec(), obstacle_trajectory.timestamps.back()));
                }
                else
                {
                    getPoseFromTime(obs.second.state._poses[0], obstacle_trajectory, (ros::Time::now() - _start_time).toSec());
                }
            }
        }

        for (auto& hum : humans)
        {
            hum_it = _human_trajectories.find(hum.second._id);

            if (hum_it != _human_trajectories.end())
            {
                // We have a trajectory for that one
                const robot_misc::HumanTrajectory& human_trajectory = hum_it->second;

                if (human_trajectory.loop)
                {
                    getHumanPosesFromTime(hum.second, human_trajectory,
                                          std::fmod((ros::Time::now() - _start_time).toSec(), human_trajectory.timestamps.back()));
                }
                else
                {
                    getHumanPosesFromTime(hum.second, human_trajectory, (ros::Time::now() - _start_time).toSec());
                }

                if (_prediction_mode.compare("SPL") == 0)  // For SPL mode
                {
                    Eigen::Quaterniond quat(0.5, 0.5, 0.5, 0.5);
                    Eigen::Matrix3d mocap = quat.toRotationMatrix();
                    std::vector<double> relOrientations;

                    for (double i = 0; i < 120; ++i)  // Trajectory for 2 seconds
                    {
                        std::vector<Eigen::Matrix4d> poses(9, Eigen::Matrix4d::Zero());
                        Eigen::Vector<double, 11> angles = Eigen::Vector<double, 11>::Zero();
                        std::vector<Eigen::Matrix3d> tmp(9, Eigen::Matrix3d::Zero());

                        // need to get back poses because we have to calculate the relative orientations
                        poses    = returnHumanPosesFromTime(hum.second, human_trajectory, (ros::Time::now() - _start_time).toSec() - (i / 60));
                        angles   = returnHumanAnglesFromTime(hum.second, human_trajectory, (ros::Time::now() - _start_time).toSec() - (i / 60));
                        poses[0] = poses[0] * robot_misc::Common::rotx(angles(0));

                        // Attention order needs to be: Hip-Neck-head_LUArm-RUArm-LFArm-RFArm-LHand-RHand but reversed because of order in vector
                        // Arrange the relative Orientations of the human trajectory
                        tmp[8] = poses[0].block<3, 3>(0, 0);
                        tmp[7] = (poses[0].block<3, 3>(0, 0)).transpose() * (poses[1].block<3, 3>(0, 0));
                        tmp[6] = robot_misc::Common::rotx(angles(1)).block<3, 3>(0, 0);
                        tmp[5] = (poses[0].block<3, 3>(0, 0)).transpose() * (poses[3].block<3, 3>(0, 0));
                        tmp[4] = (poses[0].block<3, 3>(0, 0)).transpose() * (poses[6].block<3, 3>(0, 0));
                        tmp[3] = (poses[3].block<3, 3>(0, 0)).transpose() * (poses[4].block<3, 3>(0, 0));
                        tmp[2] = (poses[6].block<3, 3>(0, 0)).transpose() * (poses[7].block<3, 3>(0, 0));
                        tmp[1] = (poses[4].block<3, 3>(0, 0)).transpose() * (poses[5].block<3, 3>(0, 0));
                        tmp[0] = (poses[7].block<3, 3>(0, 0)).transpose() * (poses[8].block<3, 3>(0, 0));

                        // Fill up matrix for relative orientations message
                        for (int j = 0; j < 9; ++j)
                        {
                            Eigen::Matrix3d rowMajor = tmp[j].transpose();
                            for (int k = 0; k < 9; ++k)
                            {
                                relOrientations.insert(relOrientations.begin(), rowMajor(8 - k));
                            }
                        }
                    }
                    MsgRelativeOrientation MsgRelOr;
                    MsgRelOr.data = relOrientations;
                    _rel_orient_publisher.publish(MsgRelOr);
                }
            }
        }
    }
    return true;
}

bool AutomaticObstacleSimulatorProcess::initialize()
{
    _start = !_wait_for_service;

    ros::NodeHandle n("~");

    _service_server        = n.advertiseService("start_simulation", &AutomaticObstacleSimulatorProcess::startSimulationCallback, this);
    _trajectory_subscriber = n.subscribe("obstacle_trajectories", 1, &AutomaticObstacleSimulatorProcess::obstacleTrajcetoryCallback, this);
    _initialized           = true;

    return true;
}

void AutomaticObstacleSimulatorProcess::setObstacleTrajectories(const std::map<int, robot_misc::ObstacleTrajectory>& obstacle_trajectories)
{
    _obstacle_trajectories = obstacle_trajectories;
}

void AutomaticObstacleSimulatorProcess::setHumanTrajectories(const std::map<int, robot_misc::HumanTrajectory>& human_trajectories)
{
    _human_trajectories = human_trajectories;
}

bool AutomaticObstacleSimulatorProcess::startSimulationCallback(SrvStartSimulation::Request& request, SrvStartSimulation::Response& response)
{

    _start = request.start;
    std_msgs::Bool data;
    data.data = request.start;
    _start_saving_publisher.publish(data);
    response.result = true;
    return true;
}

void AutomaticObstacleSimulatorProcess::obstacleTrajcetoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    std::vector<std::map<int, robot_misc::ObstacleTrajectory>::iterator> iterators;
    std::map<int, robot_misc::ObstacleTrajectory>::iterator it;
    int id = -1;

    for (int i = 0; i < (int)msg->joint_names.size(); ++i)
    {
        id = std::stoi(msg->joint_names[i]);
        it = _obstacle_trajectories.find(id);

        if (it == _obstacle_trajectories.end())
        {
            it = (_obstacle_trajectories.insert(std::pair<int, robot_misc::ObstacleTrajectory>(id, robot_misc::ObstacleTrajectory()))).first;
            it->second.id = id;
        }
        it->second.timestamps.clear();
        it->second.poses.clear();
        iterators.push_back(it);
    }

    // Number of ids and number of trajectories must match
    assert(msg->points[0].transforms.size() == msg->joint_names.size());
    assert(iterators.size() == msg->joint_names.size());

    // Fill trajectories
    for (int i = 0; i < (int)msg->points.size(); ++i)
    {
        for (int k = 0; k < (int)iterators.size(); ++k)
        {
            robot_misc::Common::poseMsgToEigen(pose, msg->points[i].transforms[k]);

            iterators[k]->second.poses.push_back(pose);
            iterators[k]->second.timestamps.push_back(msg->points[i].time_from_start.toSec());
        }
    }
}

void AutomaticObstacleSimulatorProcess::getPoseFromTime(Eigen::Ref<Eigen::Matrix4d> pose, const robot_misc::ObstacleTrajectory& trajectory,
                                                        double sim_time)
{
    if (sim_time > trajectory.timestamps.back())
    {
        pose = trajectory.poses.back();
        return;
    }

    if (sim_time < time_zero)
    {
        pose = trajectory.poses.front();
        return;
    }

    for (int i = 0; i < (int)trajectory.timestamps.size(); ++i)
    {
        if (std::fabs(trajectory.timestamps[i] - sim_time) < time_zero)
        {
            pose = trajectory.poses[i];
            break;
        }

        if (sim_time < trajectory.timestamps[i])
        {
            robot_misc::Common::interpolatePose(
                pose, trajectory.poses[i - 1], trajectory.poses[i],
                (sim_time - trajectory.timestamps[i - 1]) / (trajectory.timestamps[i] - trajectory.timestamps[i - 1]));
            break;
        }
    }
}

void AutomaticObstacleSimulatorProcess::getHumanPosesFromTime(robot_misc::Human& human, const robot_misc::HumanTrajectory& trajectory,
                                                              double sim_time)
{
    // If the simulation takes longer than the trajectory
    if (sim_time > trajectory.timestamps.back())
    {
        // Take last angle and reuse
        human._joint_angles = std::vector<double>(
            trajectory.configurations.back().data(),
            trajectory.configurations.back().data() + trajectory.configurations.back().rows() * trajectory.configurations.back().cols());

        human.updatePoses(trajectory.footprints.back());
        return;
    }

    // Before human trajectory starts
    if (sim_time < time_zero)
    {
        // take first angle of trajectory
        human._joint_angles = std::vector<double>(
            trajectory.configurations.front().data(),
            trajectory.configurations.front().data() + trajectory.configurations.front().rows() * trajectory.configurations.front().cols());

        human.updatePoses(trajectory.footprints.front());
        return;
    }

    for (int i = 0; i < (int)trajectory.timestamps.size(); ++i)
    {
        // Current time matches with time_zero precision a trajectory timestamp
        if (std::fabs(trajectory.timestamps[i] - sim_time) < time_zero)
        {
            human._joint_angles =
                std::vector<double>(trajectory.configurations[i].data(),
                                    trajectory.configurations[i].data() + trajectory.configurations[i].rows() * trajectory.configurations[i].cols());

            human.updatePoses(trajectory.footprints[i]);
            break;
        }
        // current time does not mathc directly a timestamp
        if (sim_time < trajectory.timestamps[i])
        {
            Eigen::VectorXd tmpAngles = Eigen::VectorXd::Zero(11);
            Eigen::Vector3d tmpFoot   = Eigen::Vector3d::Zero();
            // add linear interpolation between timestamps
            tmpAngles =
                human.interpolateVectors(trajectory.configurations[i - 1], trajectory.configurations[i],
                                         (sim_time - trajectory.timestamps[i - 1]) / (trajectory.timestamps[i] - trajectory.timestamps[i - 1]));
            tmpFoot = human.interpolateVectors(trajectory.footprints[i - 1], trajectory.footprints[i],
                                               (sim_time - trajectory.timestamps[i - 1]) / (trajectory.timestamps[i] - trajectory.timestamps[i - 1]));

            human._joint_angles = std::vector<double>(tmpAngles.data(), tmpAngles.data() + tmpAngles.rows() * tmpAngles.cols());
            human.updatePoses(tmpFoot);
            break;
        }
    }
}

std::vector<Eigen::Matrix4d> AutomaticObstacleSimulatorProcess::returnHumanPosesFromTime(robot_misc::Human& human,
                                                                                         const robot_misc::HumanTrajectory& trajectory,
                                                                                         double sim_time)
{
    Eigen::Vector<double, 11> angles = Eigen::Vector<double, 11>::Zero();
    std::vector<Eigen::Matrix4d> poses(9, Eigen::Matrix4d::Zero());
    if (sim_time > trajectory.timestamps.back())
    {
        angles = trajectory.configurations.back();

        poses = human.returnPoses(trajectory.footprints.back(), angles);
        return poses;
    }

    if (sim_time < time_zero)
    {
        angles = trajectory.configurations.front();

        poses = human.returnPoses(trajectory.footprints.front(), angles);
        return poses;
    }

    for (int i = 0; i < (int)trajectory.timestamps.size(); ++i)
    {
        if (std::fabs(trajectory.timestamps[i] - sim_time) < time_zero)
        {
            angles = trajectory.configurations[i];

            poses = human.returnPoses(trajectory.footprints[i], angles);
            break;
        }

        if (sim_time < trajectory.timestamps[i])
        {
            Eigen::VectorXd tmpAngles = Eigen::VectorXd::Zero(11);
            Eigen::Vector3d tmpFoot   = Eigen::Vector3d::Zero();
            tmpAngles =
                human.interpolateVectors(trajectory.configurations[i - 1], trajectory.configurations[i],
                                         (sim_time - trajectory.timestamps[i - 1]) / (trajectory.timestamps[i] - trajectory.timestamps[i - 1]));
            tmpFoot = human.interpolateVectors(trajectory.footprints[i - 1], trajectory.footprints[i],
                                               (sim_time - trajectory.timestamps[i - 1]) / (trajectory.timestamps[i] - trajectory.timestamps[i - 1]));

            poses = human.returnPoses(tmpFoot, tmpAngles);
            break;
        }
    }
    return poses;
}

Eigen::VectorXd AutomaticObstacleSimulatorProcess::returnHumanAnglesFromTime(robot_misc::Human& human, const robot_misc::HumanTrajectory& trajectory,
                                                                             double sim_time)
{
    Eigen::Vector<double, 11> angles = Eigen::Vector<double, 11>::Zero();
    if (sim_time > trajectory.timestamps.back())
    {
        angles = trajectory.configurations.back();
        return angles;
    }

    if (sim_time < time_zero)
    {
        angles = trajectory.configurations.front();

        return angles;
    }

    for (int i = 0; i < (int)trajectory.timestamps.size(); ++i)
    {
        if (std::fabs(trajectory.timestamps[i] - sim_time) < time_zero)
        {
            angles = trajectory.configurations[i];
            break;
        }

        if (sim_time < trajectory.timestamps[i])
        {
            Eigen::VectorXd tmpAngles = Eigen::VectorXd::Zero(11);
            tmpAngles =
                human.interpolateVectors(trajectory.configurations[i - 1], trajectory.configurations[i],
                                         (sim_time - trajectory.timestamps[i - 1]) / (trajectory.timestamps[i] - trajectory.timestamps[i - 1]));
            break;
        }
    }
    return angles;
}

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot
