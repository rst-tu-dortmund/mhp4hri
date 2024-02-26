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

#ifndef STATE_EXTRAPOLATION_PROCESS_H
#define STATE_EXTRAPOLATION_PROCESS_H

#include <mhp_robot/MsgExtrapolatedAngle.h>
#include <mhp_robot/MsgFutureAngles.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/base_process.h>
#include <chrono>

namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

class StateExtrapolationProcess : public BaseProcess
{
 public:
    using Ptr  = std::shared_ptr<StateExtrapolationProcess>;
    using UPtr = std::unique_ptr<StateExtrapolationProcess>;

    StateExtrapolationProcess(const std::string& name, double dt, int N, int extrapolationSteps = 5, bool constant_velocity_filter = false);

    bool process(std::map<int, robot_misc::Obstacle>& static_obstacles, std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                 std::map<int, robot_misc::Human>& humans, std::map<int, robot_misc::UtilityObject>& utility_objects,
                 std::map<int, robot_misc::Plane>& planes, bool forced = false) override;

 private:
    double _dt                        = 0.1;
    int _N                            = 20;
    int _extrapolation_steps          = 5;
    static constexpr double norm_zero = 0.001;  // ~0.1 degree precision

    void extrapolate(robot_misc::Obstacle& obstacle) const;
    void extrapolateFootPrint(robot_misc::Human& human, int zerOrderHoldStep) const;
    void extrapolateJointSpace(robot_misc::Human& human, int zeroOrderHoldStep) const;

    ros::Publisher _future_angle_pub;
    ros::Publisher _all_poses_extrapolation_pub;
    ros::Publisher _all_extrapolated_foot_prints;
    ros::Publisher _extrapolation_time_publisher;

    std::chrono::high_resolution_clock::time_point _begin_time_measurement;
    std::chrono::high_resolution_clock::time_point _end_time_measurement;

    bool _stop_extrapolation_time = true;
    bool _const_vel_model         = false;

    std::map<int, std::string> _frame_names = {{0, "Hip"},   {1, "Neck"},  {2, "Head"},  {3, "LUArm"}, {4, "LFArm"},
                                               {5, "LHand"}, {6, "RUArm"}, {7, "RFArm"}, {8, "RHand"}, {9, "Leg"}};

    ros::NodeHandle _nh;
};

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // STATE_EXTRAPOLATION_PROCESS_H
