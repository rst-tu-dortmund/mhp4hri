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

#ifndef AUTOMATIC_OBSTACLE_SIMULATOR_PROCESS_H
#define AUTOMATIC_OBSTACLE_SIMULATOR_PROCESS_H

#include <mhp_robot/SrvStartSimulation.h>
#include <mhp_robot/robot_misc/human_trajectory.h>
#include <mhp_robot/robot_misc/obstacle_trajectory.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/base_process.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <map>

namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

class AutomaticObstacleSimulatorProcess : public BaseProcess
{
 public:
    using Ptr  = std::shared_ptr<AutomaticObstacleSimulatorProcess>;
    using UPtr = std::unique_ptr<AutomaticObstacleSimulatorProcess>;

    AutomaticObstacleSimulatorProcess(const std::string& name, bool wait_for_service = true, const std::string prediction_mode = "None");

    bool process(std::map<int, robot_misc::Obstacle>& static_obstacles, std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                 std::map<int, robot_misc::Human>& humans, std::map<int, robot_misc::UtilityObject>& utility_objects,
                 std::map<int, robot_misc::Plane>& planes, bool forced = false) override;

    bool initialize() override;

    void setObstacleTrajectories(const std::map<int, robot_misc::ObstacleTrajectory>& obstacle_trajectories);
    void setHumanTrajectories(const std::map<int, robot_misc::HumanTrajectory>& human_trajectories);

 private:
    std::map<int, robot_misc::ObstacleTrajectory> _obstacle_trajectories;
    std::map<int, robot_misc::HumanTrajectory> _human_trajectories;

    bool _first_start                 = true;
    bool _start                       = false;
    bool _wait_for_service            = true;
    static constexpr double time_zero = 0.001;  // 1ms precision
    std::string _prediction_mode      = "None";

    ros::Publisher _rel_orient_publisher;
    ros::Publisher _start_saving_publisher;

    ros::Time _start_time;
    ros::ServiceServer _service_server;
    ros::Subscriber _trajectory_subscriber;

    bool startSimulationCallback(SrvStartSimulation::Request& request, SrvStartSimulation::Response& response);
    void obstacleTrajcetoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);

    void getPoseFromTime(Eigen::Ref<Eigen::Matrix4d> pose, const robot_misc::ObstacleTrajectory& trajectory, double sim_time);
    void getHumanPosesFromTime(robot_misc::Human& human, const robot_misc::HumanTrajectory& trajectory, double sim_time);
    std::vector<Eigen::Matrix4d> returnHumanPosesFromTime(robot_misc::Human& human, const robot_misc::HumanTrajectory& trajectory, double sim_time);
    Eigen::VectorXd returnHumanAnglesFromTime(robot_misc::Human& human, const robot_misc::HumanTrajectory& trajectory, double sim_time);

    ros::NodeHandle _nh;
};

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // AUTOMATIC_OBSTACLE_SIMULATOR_PROCESS_H
