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

#ifndef STATE_ESTIMATION_PROCESS_H
#define STATE_ESTIMATION_PROCESS_H

#include <mhp_robot/robot_misc/common.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/base_process.h>
#include <mhp_robot/robot_obstacle/state_estimators/base_state_estimator_joint_space.h>
#include <mhp_robot/robot_obstacle/state_estimators/base_state_estimator_task_space.h>
#include <chrono>

namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

class StateEstimationProcess : public BaseProcess
{
 public:
    using Ptr  = std::shared_ptr<StateEstimationProcess>;
    using UPtr = std::unique_ptr<StateEstimationProcess>;

    StateEstimationProcess(const std::string& name);

    bool process(std::map<int, robot_misc::Obstacle>& static_obstacles, std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                 std::map<int, robot_misc::Human>& humans, std::map<int, robot_misc::UtilityObject>& utility_objects,
                 std::map<int, robot_misc::Plane>& planes, bool forced = false) override;

    void setEstimatorsTaskSpace(std::map<int, state_estimators::BaseStateEstimatorTaskSpace::UPtr>& estimators);
    void setEstimatorsJointSpace(std::map<int, state_estimators::BaseStateEstimatorJointSpace::UPtr>& estimators);

 private:
    std::map<int, state_estimators::BaseStateEstimatorTaskSpace::UPtr> _estimators_task_space;
    std::map<int, state_estimators::BaseStateEstimatorJointSpace::UPtr> _estimators_joint_space;

    Eigen::Matrix3d _mocap_rot = (Eigen::Matrix3d() << 0, 0, 1, 1, 0, 0, 0, 1, 0).finished();

    ros::Publisher _current_angles_publisher;
    ros::Publisher _state_vector_publisher;
    ros::Publisher _estimation_time_publisher;

    std::chrono::high_resolution_clock::time_point _begin_time_measurement;
    std::chrono::high_resolution_clock::time_point _end_time_measurement;

    bool _stop_estimation_time = true;

    ros::NodeHandle _nh;
};

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // STATE_ESTIMATION_PROCESS_H
