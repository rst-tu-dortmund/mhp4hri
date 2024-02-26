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

#ifndef HUMAN_MOTION_PREDICTION_PROCESS_H
#define HUMAN_MOTION_PREDICTION_PROCESS_H

#include <mhp_robot/MsgDistances.h>
#include <mhp_robot/MsgRelativeOrientation.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/base_process.h>
#include <ros/ros.h>

namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

class HumanMotionPredictionProcess : public BaseProcess
{
 public:
    using Ptr  = std::shared_ptr<HumanMotionPredictionProcess>;
    using UPtr = std::unique_ptr<HumanMotionPredictionProcess>;

    HumanMotionPredictionProcess(const std::string& name);

    bool process(std::map<int, robot_misc::Obstacle>& static_obstacles, std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                 std::map<int, robot_misc::Human>& humans, std::map<int, robot_misc::UtilityObject>& utility_objects,
                 std::map<int, robot_misc::Plane>& planes, bool forced = false) override;

    void callback_prediction(const mhp_robot::MsgRelativeOrientation& msg);
    void callback_distance(const mhp_robot::MsgDistances& msg);
    int nearest_neighbour(const robot_misc::Human& human, Eigen::Ref<Eigen::Vector3d> hip_position, const Eigen::Ref<Eigen::MatrixXd> pos_dist,
                          const Eigen::Ref<Eigen::MatrixXd> rot_dist);

 private:
    ros::Subscriber _prediction_subscriber;
    ros::Subscriber _dist_listener;

    int _prediction_available_flag = 0;
    // saves the predictions for the next timesteps in an 10 Hz intervall
    // first map element is the prediction step
    // second map element is an Matrix which includes among themselves the poses for each joint
    std::unordered_map<int, Eigen::Matrix<double, 36, 4>> _future_poses;
    int _prediction_time_planner = 3;  // Time in seconds for which the robot motion planner is working

    ros::NodeHandle _nh;
};

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // HUMAN_MOTION_PREDICTION_PROCESS_H
