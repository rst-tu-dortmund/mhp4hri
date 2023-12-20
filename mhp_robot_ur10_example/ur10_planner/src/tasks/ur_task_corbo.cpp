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

#include <nav_msgs/Path.h>
#include <ur10_planner/tasks/ur_task_corbo.h>

namespace mhp_planner {

TaskInterface::Ptr URTask::getInstance() const { return std::make_shared<URTask>(); }

void URTask::performTask(Environment& environment, std::string* err_msg)
{
    /*
     * Initializing
     */

    // get planner and robot
    ControllerInterface* planner = environment.getController().get();
    PlantInterface* robot        = environment.getPlant().get();

    // create helper classes
    _ur_kinematic = std::make_unique<URKinematic>();
    _ur_utility   = std::make_unique<URUtility>();

    // setup important time stamps and durations
    Time tf(_sim_time);           // simulation time
    Time t_measured_state(0);     // time of measured state
    Time t_compensator_start(0);  // start time of computation time measurements
    Time t_control(0);            // time of sent control
    Duration dt(_dt);             // planning dt
    Rate rate(dt);                // planning rate

    double main_obj               = MHP_PLANNER_INF_DBL;
    double compensation_delay     = 0.0;
    double measured_planning_time = 0.0;
    double computation_time       = 0.0;
    bool trigger_replan           = false;
    bool planned                  = false;

    // create container to pass delays to collision classes
    mhp_robot::robot_misc::PlanningDelay planning_delay;

    // data for publishing the predicted states in task space
    Eigen::Quaterniond ee_quaternion;
    Eigen::Quaterniond quaternion_ref;
    Eigen::VectorXd s_error = Eigen::VectorXd::Zero(4);

    // data for
    Eigen::VectorXd compensated_state = Eigen::VectorXd::Zero(robot->getOutputDimension());
    Eigen::VectorXd measured_state    = Eigen::VectorXd::Zero(robot->getOutputDimension());
    Eigen::VectorXd ee_pose           = Eigen::VectorXd::Zero(7);  // end-effector pose [p_x,p_y,p_y,q_w,q_x,q_y,q_z]

    // time series for planned controls and states
    TimeSeries::Ptr optimized_controls = std::make_shared<TimeSeries>();
    TimeSeries::Ptr optimized_states   = std::make_shared<TimeSeries>();
    optimized_controls->add(0, Eigen::VectorXd::Zero(planner->getControlInputDimension()));

    // init references
    initReferences(robot->getOutputDimension(), planner->getControlInputDimension());

    // verify environment
    if (!verify(environment, err_msg)) return;

    if (_dt <= 0 && !planner->supportsAsynchronousControl())
    {
        PRINT_ERROR(
            "URTask: dt <= 0 selected, but current controller does "
            "not support asynchronous control.");
        return;
    }

    // initialize robot
    if (!robot->initialize())
    {
        PRINT_ERROR("URTask: Robot initialization failed.");
        return;
    }

    // initialize main planner
    if (!planner->initialize(measured_state, *_state_reference, *_control_reference, dt, Time(0), _pose_reference.get()))
    {
        PRINT_ERROR("URTask: Planner initialization failed.");
        return;
    }

    // initialize ros subscriber for references
    ros::NodeHandle n("~");

    // use root ns because we generally do not know the source topic
    std::string state_target_topic            = "/state_targets";
    std::string task_space_target_topic       = "/task_space_targets";
    std::string dynamic_obstacle_update_topic = "/robot_workspace_monitor/obstacles";
    std::string start_envrionment_service     = "/bag_parser/robot_tf_bag_parser/start_simulation";
    std::string prediction_marker_topic       = "prediction";

    ros::Subscriber state_target_sub = n.subscribe(state_target_topic, 1, &URTask::stateTargetCallback, this, ros::TransportHints().tcpNoDelay());
    PRINT_INFO("URTask:  listening for state targets on topic " << state_target_topic << ".");

    ros::Subscriber task_space_target_sub =
        n.subscribe(task_space_target_topic, 1, &URTask::taskSpaceTargetCallback, this, ros::TransportHints().tcpNoDelay());
    PRINT_INFO("URTask:  listening for custom targets on topic " << task_space_target_topic << ".");

    ros::Subscriber virtual_obstacle_sub =
        n.subscribe(dynamic_obstacle_update_topic, 1, &URTask::obstacleCallback, this, ros::TransportHints().tcpNoDelay());
    PRINT_INFO("URTask:  listening for virtual dynamic obstacles on topic " << dynamic_obstacle_update_topic << ".");

    ros::ServiceClient environment_svc_client = n.serviceClient<mhp_robot::SrvStartSimulation>(start_envrionment_service);

    _prediction_pub = n.advertise<nav_msgs::Path>(prediction_marker_topic, 1, true);

    // wait for environment info
    ROS_INFO("URTask: Waiting up to 1 second for environment information...");

    mhp_robot::MsgObstacleListConstPtr obstacle_msg =
        ros::topic::waitForMessage<mhp_robot::MsgObstacleList>(dynamic_obstacle_update_topic, n, ros::Duration(1));

    if (!obstacle_msg) ROS_WARN("URTask: No environment information recieved!");

    // start obstacle simulator
    startEnvironment(true, environment_svc_client);

    /*
     * PLANNING LOOP START
     */

    // store start time to bias planning to 0
    Time t_start = Time::now();
    rate.reset();

    while ((Time::now() - t_start).toTime() <= tf && ok())
    {
        // check for multiple target callbacks
        if (_new_sref && _new_xref)
        {
            PRINT_ERROR("URTask: Multiple target types detected. Task will stop.");
            // shutdown experiment
            robot->stop();
            return;
        }

        /*
         * MEASURE CURRENT STATE
         */

        // take time of state measurement and compensator start
        t_compensator_start = Time::now();
        t_measured_state    = (Time::now() - t_start).toTime();

        // measure current state
        if (!robot->output(measured_state, t_measured_state)) PRINT_ERROR("URTask: error while retrieving plant output.");

        // the compensated state equals the measured state in case of no
        // compensation
        compensated_state = measured_state;

        /*
         * PARSE REFERENCES
         */

        // assign references from callbacks to actual reference container
        processRefsAndInits(measured_state, *optimized_states, t_measured_state);

        /*
         * PLANNING TRIGGER
         */

        /*
         * COMPENSATE DELAYS
         */

        // are we going to plan?
        if (_trigger_replan || !_state_reference->isStatic())
        {
            // get computation time (measured or fixed value)
            computation_time = (_computation_delay < 0) ? measured_planning_time : _computation_delay;
            // add offset_delay if desired
            compensation_delay = computation_time + _offset_delay;

            // compensate delay of measured state for next planning by forward
            // integration of future controls
            robot->getFutureState(t_measured_state, measured_state, compensation_delay, compensated_state);

            /*
             * PLAN
             */

            // pass prediction time to collision classes
            planning_delay._mutex.lock();
            planning_delay._delay = compensation_delay;
            planning_delay._mutex.unlock();

            // plan
            if (!planner->step(compensated_state, *_state_reference, *_control_reference, dt, Time(t_measured_state.toSec() + compensation_delay),
                               optimized_controls, optimized_states, _pose_reference.get(), _state_init.get(), _control_init.get(), "", _reinit))
            {
                PRINT_ERROR("URTask: controller error.");
            }

            main_obj = planner->getObjectiveValue();
            _reinit  = false;

            // we have planned
            planned = true;

            /*
             * EXECUTE PLAN
             */

            // pass new trajectory to plant
            t_control = (Time::now() - t_start).toTime();
            robot->control(optimized_controls, optimized_states, dt, t_control);
        }

        // store time required for the task iteration
        _measured_planning_time = (Time::now() - t_compensator_start).toSec();

        // apply filter to computation time if the latter is not fixed and we did
        // actually plan
        if (planned && _computation_delay < 0 && _computation_delay_filter)
        {
            measured_planning_time = _computation_delay_filter->filter(t_control.toSec(), _measured_planning_time);
        }

        // send prediction marker
        if (_publish_prediction_marker) publishPredictionMarker(*optimized_states);

        // if dt<=0 -> inherit from controller (asynchronous control mode)
        if (_dt <= 0)
        {
            if (planner->getControlDuration() > 0)
            {
                dt.fromSec(planner->getControlDuration());
                rate.updateCycleTime(dt);
            }
            else
                PRINT_ERROR(
                    "URTask: asychnronous control mode: controller "
                    "returned dt<=0.");
        }

        // reset flags for next iteration
        planned = false;

        ros::spinOnce();  // process callbacks

        if (!rate.sleep())
        {
            PRINT_WARNING("URTask(): rate exceeded (" << rate.lastCycleTime().toSec() << "s/" << dt << "s).");
        }

        if (_single_shot)
        {
            ros::Duration(2.0).sleep();  // let publisher send
            break;
        }
    }

    /*
     * PLANNING LOOP END
     */

    // Shutdown experiment
    robot->stop();

    // reset/stop obstacle simulator
    startEnvironment(false, environment_svc_client);
    ROS_INFO("URTask: Shutting down experiment.");
}

bool URTask::verify(const Environment& environment, std::string* msg) const
{
    bool ret_val = true;

    if (msg) msg->clear();

    // verify environment
    std::string environment_msg;
    ret_val = ret_val && environment.verify(&environment_msg);
    if (msg) *msg += environment_msg;
    if (ret_val == false) return false;  // we need all objects in environment allocated!

    if (environment.getController()->getStateDimension() != _state_reference->getDimension())
    {
        ret_val = false;
        if (msg)
            *msg += "State reference trajectory dimension (" + std::to_string(_state_reference->getDimension()) +
                    ") does not match controller state dimension (" + std::to_string(environment.getController()->getStateDimension()) + ").\n";
    }
    if (environment.getController()->getControlInputDimension() != _control_reference->getDimension())
    {
        ret_val = false;
        if (msg)
            *msg += "Control reference trajectory dimension (" + std::to_string(_control_reference->getDimension()) +
                    ") does not match controller control input dimension (" +
                    std::to_string(environment.getController()->getControlInputDimension()) + ").\n";
    }

    if (environment.getPlant()->requiresFutureControls() && !environment.getController()->providesFutureControls())
    {
        ret_val = false;
        if (msg)
            *msg +=
                "Controller does not support control sequences, that are "
                "required by the plant";
    }

    if (environment.getPlant()->requiresFutureStates() && !environment.getController()->providesFutureStates())
    {
        ret_val = false;
        if (msg)
            *msg +=
                "Controller does not support state sequences, that are required "
                "by the plant";
    }

    return ret_val;
}

void URTask::reset() {}

void URTask::stateTargetCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
    PRINT_INFO_ONCE("URTask: Received state target.");

    if (msg->joint_names.size() != _ur_utility->getJointsCount() || msg->points.size() == 0)
    {
        PRINT_ERROR_ONCE(
            "URTask: Invalid number of Joint names or no actual "
            "target points.");
        return;
    }

    TimeSeries state_ts(_state_reference_callback->getDimension());
    TimeSeries control_ts(_control_reference_callback->getDimension());
    Eigen::VectorXd xref = Eigen::VectorXd::Zero(_state_reference_callback->getDimension());
    Eigen::VectorXd uref = Eigen::VectorXd::Zero(_control_reference_callback->getDimension());

    std::vector<double> joint_positions(_ur_utility->getJointsCount(), 0.0);
    std::vector<double> joint_velocities(_ur_utility->getJointsCount(), 0.0);

    if (_first_xref)
    {
        if (!_ur_utility->initJointMapping(msg->joint_names))
        {
            return;
        }
        _first_xref = false;
    }

    for (uint i = 0; i < msg->points.size(); ++i)
    {
        if (msg->points[i].positions.size() != _ur_utility->getJointsCount())
        {
            PRINT_ERROR_ONCE("URTask: Not enough joint values.");
            return;
        }

        // Parse msg
        _ur_utility->parseJointStates(joint_positions, msg->points[i].positions);
        xref = Eigen::Map<const Eigen::VectorXd>(joint_positions.data(), joint_positions.size());

        // Add point
        state_ts.add(msg->points[i].time_from_start.toSec(), xref);

        if (msg->points[i].velocities.size() == _ur_utility->getJointsCount())
        {
            _ur_utility->parseJointStates(joint_velocities, msg->points[i].velocities);
            uref = Eigen::Map<const Eigen::VectorXd>(joint_velocities.data(), joint_velocities.size());
            control_ts.add(msg->points[i].time_from_start.toSec(), uref);
        }
        else
        {
            PRINT_INFO_ONCE(
                "URTask: Joint space target message does not contain consistent "
                "velocities. Assuming zero control reference.");
            control_ts.add(msg->points[i].time_from_start.toSec(), Eigen::VectorXd::Zero(_ur_utility->getJointsCount()));
        }
    }

    // Set new target
    _state_reference_callback->setTrajectory(state_ts);
    _control_reference_callback->setTrajectory(control_ts);

    _new_xref = true;
}

void URTask::taskSpaceTargetCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
    if (msg->joint_names.size() != 1 || msg->joint_names[0].compare("ee_joint") != 0 || msg->points.size() == 0)
    {
        PRINT_ERROR_ONCE(
            "URTask: Task space target message does not have the required "
            "characteristics. Ignoring custom target message.");
        return;
    }

    TimeSeries task_ts(_pose_reference_callback->getDimension());
    Eigen::Matrix<double, 13, 1> sref;

    for (uint i = 0; i < msg->points.size(); ++i)
    {
        if (msg->points[i].transforms.size() != 1)
        {
            PRINT_ERROR_ONCE(
                "URTask: Task space target message does not have the required "
                "characteristics. Ignoring custom target message.");
            return;
        }

        // Parse msg
        sref(0)  = msg->points[i].transforms[0].translation.x;
        sref(1)  = msg->points[i].transforms[0].translation.y;
        sref(2)  = msg->points[i].transforms[0].translation.z;
        sref(3)  = msg->points[i].transforms[0].rotation.w;
        sref(4)  = msg->points[i].transforms[0].rotation.x;
        sref(5)  = msg->points[i].transforms[0].rotation.y;
        sref(6)  = msg->points[i].transforms[0].rotation.z;
        sref(7)  = msg->points[i].velocities[0].linear.x;
        sref(8)  = msg->points[i].velocities[0].linear.y;
        sref(9)  = msg->points[i].velocities[0].linear.z;
        sref(10) = msg->points[i].velocities[0].angular.x;
        sref(11) = msg->points[i].velocities[0].angular.y;
        sref(12) = msg->points[i].velocities[0].angular.z;

        // Add point
        task_ts.add(msg->points[i].time_from_start.toSec(), sref);
    }

    // Set new target
    _pose_reference_callback->setTrajectory(task_ts);

    _new_sref = true;
}

void URTask::obstacleCallback(const mhp_robot::MsgObstacleListConstPtr& msg)
{
    PRINT_INFO_ONCE("URTask: Received virtual dynamic obstacle pose.");

    _obstacle_manager._mutex.lock();
    Common::parseObstacleMsg(msg, _obstacle_manager._static_obstacles, _obstacle_manager._dynamic_obstacles, _obstacle_manager._humans,
                             _obstacle_manager._planes);
    _obstacle_manager._mutex.unlock();
}

void URTask::processRefsAndInits(const Eigen::Ref<const Eigen::VectorXd>& measured_state, const TimeSeries& optimized_states,
                                 const Time& t_measured_state)
{
    if (_new_sref)
    {
        // copy to main reference
        _pose_reference = std::make_shared<DiscreteTimeReferenceTrajectory>(*_pose_reference_callback);
        _pose_reference->setTimeFromStart(t_measured_state);

        _reinit   = true;
        _new_sref = false;
    }

    if (_new_xref)
    {
        // copy to main reference
        _state_reference   = std::make_shared<DiscreteTimeReferenceTrajectory>(*_state_reference_callback);
        _control_reference = std::make_shared<DiscreteTimeReferenceTrajectory>(*_control_reference_callback);

        _state_reference->setTimeFromStart(t_measured_state);
        _control_reference->setTimeFromStart(t_measured_state);

        _reinit   = true;
        _new_xref = false;
    }

    // create initializations

    _state_init   = std::make_shared<DiscreteTimeReferenceTrajectory>(*_state_reference);
    _control_init = std::make_shared<DiscreteTimeReferenceTrajectory>(*_control_reference);
}

void URTask::initReferences(int state_dimension, int control_dimension)
{
    assert(_initial_state_reference.rows() == state_dimension);

    Eigen::Quaterniond ee_quaternion;
    Eigen::VectorXd initial_pose_reference                                = Eigen::VectorXd::Zero(13);
    const Eigen::Ref<const Eigen::Matrix<double, 4, 4>> ee_transformation = _ur_kinematic->getEndEffectorMatrix(_initial_state_reference);

    ee_quaternion                    = ee_transformation.block<3, 3>(0, 0);
    initial_pose_reference.head<3>() = ee_transformation.block<3, 1>(0, 3);
    initial_pose_reference(3)        = ee_quaternion.w();
    initial_pose_reference(4)        = ee_quaternion.x();
    initial_pose_reference(5)        = ee_quaternion.y();
    initial_pose_reference(6)        = ee_quaternion.z();

    TimeSeries state_ts(state_dimension);
    TimeSeries state_ts_p(state_dimension);
    TimeSeries state_ts_cb(state_dimension);
    TimeSeries control_ts(control_dimension);
    TimeSeries control_ts_p(control_dimension);
    TimeSeries control_ts_cb(control_dimension);
    TimeSeries task_space_ts(13);
    TimeSeries task_space_ts_p(13);
    TimeSeries task_space_ts_cb(13);

    state_ts.add(0, _initial_state_reference);
    state_ts_p.add(0, _initial_state_reference);
    state_ts_cb.add(0, _initial_state_reference);
    control_ts.add(0, Eigen::VectorXd::Zero(control_dimension));
    control_ts_p.add(0, Eigen::VectorXd::Zero(control_dimension));
    control_ts_cb.add(0, Eigen::VectorXd::Zero(control_dimension));
    task_space_ts.add(0, initial_pose_reference);
    task_space_ts_p.add(0, initial_pose_reference);
    task_space_ts_cb.add(0, initial_pose_reference);

    _state_reference            = std::make_shared<DiscreteTimeReferenceTrajectory>(state_ts, TimeSeries::Interpolation::Linear);
    _state_reference_callback   = std::make_shared<DiscreteTimeReferenceTrajectory>(state_ts_cb, TimeSeries::Interpolation::Linear);
    _control_reference          = std::make_shared<DiscreteTimeReferenceTrajectory>(control_ts, TimeSeries::Interpolation::Linear);
    _control_reference_callback = std::make_shared<DiscreteTimeReferenceTrajectory>(control_ts_cb, TimeSeries::Interpolation::Linear);
    _pose_reference             = std::make_shared<DiscreteTimeReferenceTrajectory>(task_space_ts, TimeSeries::Interpolation::ZeroOrderHold);
    _pose_reference_callback    = std::make_shared<DiscreteTimeReferenceTrajectory>(task_space_ts_cb, TimeSeries::Interpolation::ZeroOrderHold);

    _state_init   = std::make_shared<DiscreteTimeReferenceTrajectory>();
    _control_init = std::make_shared<DiscreteTimeReferenceTrajectory>();
}

void URTask::publishPredictionMarker(const TimeSeries& sequence)
{
    const std::vector<double>& time = sequence.getTime();
    nav_msgs::Path path_msg;

    for (int i = 0; i < (int)time.size(); ++i)
    {
        geometry_msgs::PoseStamped pose_stamped_msg;

        Common::poseEigenToMsg(_ur_kinematic->getEndEffectorMatrix(sequence.getValuesMap(i)), pose_stamped_msg.pose);
        path_msg.poses.push_back(pose_stamped_msg);
    }

    path_msg.header.stamp    = ros::Time::now();
    path_msg.header.frame_id = _ur_utility->getRobotDescription().getFirstFrame();

    _prediction_pub.publish(path_msg);
}

void URTask::startEnvironment(bool start, ros::ServiceClient& client)
{
    if (_start_environment)
    {
        mhp_robot::SrvStartSimulation msg;
        msg.request.start = start;

        if (client.call(msg))
        {
            ROS_INFO_STREAM("URTask: Environment " << (start ? "started" : "stopped"));
        }
        else
        {
            ROS_WARN_STREAM("URTask: Could not " << (start ? "start" : "stop") << " environment");
        }
    }
}

bool URTask::fromParameterServer(const std::string& ns)
{
    // get node handle
    ros::NodeHandle nh;

    // get parameters
    if (!nh.getParam(ns + "/sim_time", _sim_time))
    {
        PRINT_ERROR("URTask: Could not read parameter sim_time.");
        return false;
    };
    if (!nh.getParam(ns + "/dt", _dt))
    {
        PRINT_ERROR("URTask: Could not read parameter dt.");
        return false;
    };

    _start_environment         = false;
    _publish_prediction_marker = true;
    _publish_task_space        = true;

    // compensator
    if (!nh.getParam(ns + "/computation_delay", _computation_delay))
    {
        PRINT_ERROR("URTask: Could not read parameter computation_delay.");
        return false;
    };
    if (!nh.getParam(ns + "/general_offset_delay", _offset_delay))
    {
        PRINT_ERROR("URTask: Could not read parameter offset_delay.");
        return false;
    };

    std::vector<double> initial_state_reference;
    if (!nh.getParam(ns + "/initial_goal_state", initial_state_reference))
    {
        PRINT_ERROR("URTask: Could not read parameter initial_state_reference.");
        return false;
    };

    _initial_state_reference = Eigen::Map<const Eigen::VectorXd>(initial_state_reference.data(), initial_state_reference.size());

    // get computation delay filter
    std::string computation_delay_filter_type;
    nh.getParam(ns + "/computation_delay_filter/filter_type", computation_delay_filter_type);
    if (computation_delay_filter_type == "None")
    {
        _computation_delay_filter = {};
    }
    else if (computation_delay_filter_type == "MovingAverageFilter")
    {
        FilterInterface::Ptr filter = FilterFactory::instance().create(computation_delay_filter_type);
        if (!filter)
        {
            PRINT_ERROR("URTask: Could not create computation delay filter");
            return false;
        }
        else
        {
            if (!filter->fromParameterServer(ns + "/computation_delay_filter/moving_average"))
            {
                PRINT_ERROR("URTask: Could not read computation delay filter parameters.");
                return false;
            }
            else
            {
                _computation_delay_filter = filter;
            }
        }
    }
    else if (computation_delay_filter_type == "MovingMedianFilter")
    {
        FilterInterface::Ptr filter = FilterFactory::instance().create(computation_delay_filter_type);
        if (!filter)
        {
            PRINT_ERROR("URTask: Could not create computation delay filter");
            return false;
        }
        else
        {
            if (!filter->fromParameterServer(ns + "/computation_delay_filter/moving_median"))
            {
                PRINT_ERROR("URTask: Could not read computation delay filter parameters.");
                return false;
            }
            else
            {
                _computation_delay_filter = filter;
            }
        }
    }
    else if (computation_delay_filter_type == "MovingLeastSquaresFilter")
    {
        FilterInterface::Ptr filter = FilterFactory::instance().create(computation_delay_filter_type);
        if (!filter)
        {
            PRINT_ERROR("URTask: Could not create computation delay filter of ");
            return false;
        }
        else
        {
            if (!filter->fromParameterServer(ns + "/computation_delay_filter/moving_least_squares"))
            {
                PRINT_ERROR("URTask: Could not read computation delay filter parameters.");
                return false;
            }
            else
            {
                _computation_delay_filter = filter;
            }
        }
    }
    else
    {
        PRINT_ERROR("URTask: Unknown computation delay filter of type " << computation_delay_filter_type);
        return false;
    }
    return true;
}

}  // namespace mhp_planner
