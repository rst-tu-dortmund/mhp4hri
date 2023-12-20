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

#ifndef OBSTACLE_TRAJECTORY_GENERATOR_H
#define OBSTACLE_TRAJECTORY_GENERATOR_H

#include <mhp_robot/robot_misc/human_trajectory.h>
#include <mhp_robot/robot_misc/obstacle_trajectory.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <map>

namespace mhp_robot {
namespace robot_obstacle {

class ObstacleTrajectoryGenerator
{
 public:
    ObstacleTrajectoryGenerator() = default;

    robot_misc::ObstacleTrajectory generateLinearMotion(const Eigen::Ref<const Eigen::Matrix4d>& p1, const Eigen::Ref<const Eigen::Matrix4d>& p2,
                                                        int samples, double duration);

    void generateLinearMotion(const Eigen::Ref<const Eigen::Matrix4d>& p1, const Eigen::Ref<const Eigen::Matrix4d>& p2, int samples, double duration,
                              robot_misc::ObstacleTrajectory& trajectory);

    robot_misc::ObstacleTrajectory generateCircularMotion(const Eigen::Ref<const Eigen::Matrix4d>& p1, const Eigen::Ref<const Eigen::Matrix4d>& p2,
                                                          const Eigen::Ref<const Eigen::Matrix4d>& p3, int samples, double duration);

    void generateCircularMotion(const Eigen::Ref<const Eigen::Matrix4d>& p1, const Eigen::Ref<const Eigen::Matrix4d>& p2,
                                const Eigen::Ref<const Eigen::Matrix4d>& p3, int samples, double duration,
                                robot_misc::ObstacleTrajectory& trajectory);

    robot_misc::HumanTrajectory generateHumanMotion(const Eigen::Ref<const Eigen::VectorXd>& q1, const Eigen::Ref<const Eigen::VectorXd>& q2,
                                                    const Eigen::Ref<const Eigen::Vector3d>& f1, const Eigen::Ref<const Eigen::Vector3d>& f2,
                                                    int samples, double duration, bool returnPath = false);

    void generateHumanMotion(const Eigen::Ref<const Eigen::VectorXd>& q1, const Eigen::Ref<const Eigen::VectorXd>& q2,
                             const Eigen::Ref<const Eigen::Vector3d>& f1, const Eigen::Ref<const Eigen::Vector3d>& f2, int samples, double duration,
                             robot_misc::HumanTrajectory& trajectory, bool returnPath = false);

    std::map<int, robot_misc::ObstacleTrajectory> parseObstacleTrajectories(const std::string param = "/obstacles/obstacle_trajectories");
    std::map<int, robot_misc::HumanTrajectory> parseHumanTrajectories(const std::string param = "/obstacles/human_trajectories");

 private:
    std::map<int, robot_misc::ObstacleTrajectory> createObstacleTrajectoryMap(const XmlRpc::XmlRpcValue& param);
    std::map<int, robot_misc::HumanTrajectory> createHumanTrajectoryMap(const XmlRpc::XmlRpcValue& param);
};

}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // OBSTACLE_TRAJECTORY_GENERATOR_H
