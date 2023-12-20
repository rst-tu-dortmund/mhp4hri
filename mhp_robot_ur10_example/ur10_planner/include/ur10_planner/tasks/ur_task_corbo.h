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

#ifndef UR_TASK_MHP_PLANNER_H
#define UR_TASK_MHP_PLANNER_H

#include <mhp_planner/controllers/controller_interface.h>

#include <mhp_planner/controllers/predictive_controller.h>
#include <mhp_planner/systems/filter_interface.h>
#include <mhp_planner/tasks/task_interface.h>
#include <mhp_robot/SrvStartSimulation.h>
#include <mhp_robot/robot_misc/planning_delay.h>
#include <mhp_robot/robot_obstacle/obstacle_list.h>
#include <ur_utilities/ur_collision/ur_collision.h>
#include <ur_utilities/ur_kinematic/ur_inverse_kinematic.h>
#include <ur_utilities/ur_kinematic/ur_kinematic.h>
#include <ur_utilities/ur_misc/ur_utility.h>

namespace mhp_planner {

class URTask : public TaskInterface
{
 public:
    using Ptr  = std::shared_ptr<URTask>;
    using UPtr = std::unique_ptr<URTask>;

    URTask() = default;

    TaskInterface::Ptr getInstance() const override;

    void performTask(Environment& environment, std::string* err_msg) override;

    bool verify(const Environment& environment, std::string* msg = nullptr) const override;

    void reset() override;

    bool fromParameterServer(const std::string& ns) override;

 private:
    using ObstacleList = mhp_robot::robot_obstacle::ObstacleList;
    using Common       = mhp_robot::robot_misc::Common;
    using URKinematic  = mhp_robot::robot_kinematic::URKinematic;
    using URUtility    = mhp_robot::robot_misc::URUtility;
    using URCollision  = mhp_robot::robot_collision::URCollision;

    double _measured_planning_time  = 0.0;
    double _sim_time                = 100.0;
    double _dt                      = 0.1;
    double _computation_delay       = 0.0;
    double _offset_delay            = 0.0;
    double _parallel_hysteresis     = 1.0;
    bool _start_environment         = false;
    bool _publish_prediction_marker = true;
    bool _publish_task_space        = false;
    bool _first_xref                = true;
    bool _new_sref                  = false;
    bool _new_xref                  = false;
    bool _reinit                    = false;
    bool _single_shot               = false;
    bool _trigger_replan            = true;

    FilterInterface::Ptr _computation_delay_filter;

    DiscreteTimeReferenceTrajectory::Ptr _state_init;
    DiscreteTimeReferenceTrajectory::Ptr _control_init;
    DiscreteTimeReferenceTrajectory::Ptr _state_reference, _state_reference_callback;
    DiscreteTimeReferenceTrajectory::Ptr _control_reference, _control_reference_callback;
    DiscreteTimeReferenceTrajectory::Ptr _pose_reference, _pose_reference_callback;
    Eigen::Matrix<double, 6, 1> _initial_state_reference;

    void stateTargetCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg);
    void taskSpaceTargetCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);
    void obstacleCallback(const mhp_robot::MsgObstacleListConstPtr& msg);

    void processRefsAndInits(const Eigen::Ref<const Eigen::VectorXd>& measured_state, const TimeSeries& optimized_states,
                             const Time& t_measured_state);
    void initReferences(int state_dimension, int control_dimension);
    void publishPredictionMarker(const TimeSeries& sequence);
    void startEnvironment(bool start, ros::ServiceClient& client);

    ros::Publisher _prediction_pub;

    ObstacleList _obstacle_manager;
    URUtility::UPtr _ur_utility;
    URKinematic::UPtr _ur_kinematic;
};

FACTORY_REGISTER_TASK(URTask)

}  // namespace mhp_planner

#endif  // UR_TASK_MHP_PLANNER_H
