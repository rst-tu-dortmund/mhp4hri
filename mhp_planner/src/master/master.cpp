/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020,
 *  TU Dortmund University, Institute of Control Theory and System Enginnering
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
 *  Authors: Christoph Rösmann
 *  Modifier(s)/Maintainer(s): Maximilian Krämer, Heiko Renz
 *********************************************************************/

#include <mhp_planner/master/master.h>

#include <fstream>
#include <memory>
#include <string>

namespace mhp_planner {

void Master::start()
{
    if (!_environment.hasController() && !_environment.hasPlant())
    {
        setDefault();
        ROS_INFO("MHP_planner_master: Started master with default settings");
    }
    else
        ROS_INFO("MHP_planner_master: Started master with existing settings");

    _internal_perform_pub = _nh.advertise<std_msgs::Bool>("perform_task", 1);
    _internal_perform_sub = _nh.subscribe("perform_task", 1, &Master::performTaskCallback, this);

    _task_service        = _nh.advertiseService("/start_task", &Master::taskCallback, this);
    _task_service_client = _nh.serviceClient<mhp_robot::SrvStartSimulation>("/start_task");

    _start_all_service = _nh.advertiseService("/start_all", &Master::startAllCallback, this);

    ROS_INFO("MHP_planner_master: Waiting for task to be performed");
}

void Master::setDefault()
{
    if (!loadFromParameterServer("/ur_mhp_planner"))
        PRINT_ERROR(
            "MHP_planner_master: Load from default parameter namespace not "
            "worked!");
}

bool Master::loadFromParameterServer(const std::string& namespace_name)
{
    // Get controller parameters and initialize controller
    if (!setController(namespace_name))
    {
        ROS_ERROR("MHP_planner_master: Controller not set correctly");
        return false;
    };

    // Get plant parameters and initialize plant
    if (!setPlant(namespace_name))
    {
        ROS_ERROR("MHP_planner_master: Plant not set correctly");
        return false;
    };

    // Get task parameters and initialize task
    if (!setTask(namespace_name))
    {
        ROS_ERROR("MHP_planner_master: Task not set correctly");
        return false;
    };

    return true;
}

bool Master::setPlant(const std::string& namespace_name)
{
    // Get plant parameters and initialize plant
    std::string plant_type;
    _nh.getParam(namespace_name + "/plant/plant_type", plant_type);

    if (plant_type == "URRobot")
    {
        PlantInterface::Ptr plant = PlantFactory::instance().create(plant_type);

        if (!plant)
        {
            ROS_ERROR_STREAM("MHP_planner_master: Could not create plant of type: " << plant_type);
            return false;
        }
        else
        {
            plant->fromParameterServer(namespace_name + "/plant");
            _environment.setPlant(plant);
        }
    }
    else
    {
        ROS_ERROR_STREAM("MHP_planner_master: Unknown plant type: " << plant_type);
        return false;
    }
    return true;
}

bool Master::setController(const std::string& namespace_name)
{
    // Get controller parameters and initialize controller
    std::string controller_type;
    _nh.getParam(namespace_name + "/controller/controller_type", controller_type);

    if (controller_type == "PredictiveController")
    {
        ControllerInterface::Ptr controller = ControllerFactory::instance().create(controller_type);

        if (!controller)
        {
            ROS_ERROR_STREAM("MHP_planner_master: Could not create controller of type: " << controller_type);
            return false;
        }
        else
        {
            controller->fromParameterServer(namespace_name + "/controller");
            _environment.setController(controller);
        }
    }
    else
    {
        ROS_ERROR_STREAM("MHP_planner_master: Unknown controller type: " << controller_type);
        return false;
    }
    return true;
}

bool Master::setTask(const std::string& namespace_name)
{
    // Get task parameters and initialize task
    std::string task_type;
    _nh.getParam(namespace_name + "/task/task_type", task_type);

    _task = TaskFactory::instance().create(task_type);
    if (_task)
    {
        if (!_task->fromParameterServer(namespace_name + "/task"))
        {
            ROS_ERROR("MHP_planner_master: task not set correctly");
            return false;
        }
    }
    else
    {
        ROS_ERROR_STREAM("MHP_planner_master: Unknown task type: " << task_type);
        return false;
    }
    return true;
}

void Master::performTaskCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (_start_task && msg->data)
    {
        setOk(true);
        if (_task)
        {
            _task->performTask(_environment);
        }
        else
        {
            ROS_ERROR_STREAM("MHP_planner_master: No task specified");
        }
    }
}

bool Master::taskCallback(mhp_robot::SrvStartSimulation::Request& request, mhp_robot::SrvStartSimulation::Response& response)
{
    _start_task = request.start;

    setOk(_start_task);

    std_msgs::Bool msg;
    msg.data = _start_task;
    _internal_perform_pub.publish(msg);

    response.result = _start_task;
    return true;
}

bool Master::startAllCallback(mhp_robot::SrvStartSimulation::Request& request, mhp_robot::SrvStartSimulation::Response& response)
{
    // We need the WS mode to start up the right simulation (rosbag startup or automatic trajectories)
    std::string workspace_monitor_mode;
    _nh.getParam("/robot_workspace_monitor/workspace_monitor_mode", workspace_monitor_mode);

    // Start the Obstacle replay clients
    ros::ServiceClient client;

    if (workspace_monitor_mode == "bag")
    {
        client = _nh.serviceClient<mhp_robot::SrvStartSimulation>("/bag_parser/robot_tf_bag_parser/start_simulation");
    }
    else if (workspace_monitor_mode == "automatic")
    {
        client = _nh.serviceClient<mhp_robot::SrvStartSimulation>("/robot_workspace_monitor/start_simulation");
    }

    // Setup the messages
    mhp_robot::SrvStartSimulation::Request srv_req;
    mhp_robot::SrvStartSimulation::Response srv_resp;
    srv_req.start = request.start;

    // Call the client
    client.call(srv_req, srv_resp);

    // call the planner (identical to taskCallback)
    _start_task = request.start;
    setOk(_start_task);
    std_msgs::Bool msg;
    msg.data = _start_task;
    _internal_perform_pub.publish(msg);

    response.result = (srv_resp.result && _start_task);

    return true;
}

bool Master::verifyConfig()
{
    if (_task)
    {
        PRINT_WARNING("MHP_planner_master: Verify configuration _task available");

        std::string result;
        if (_task->verify(_environment, &result))
        {
            PRINT_WARNING("MHP_planner_master: Verifiied Task");

            ROS_INFO_STREAM("MHP_planner_master: Task verified successfully");
            return true;
        }
        else
        {
            PRINT_WARNING("MHP_planner_master: Not Verifiied Task");

            ROS_ERROR_STREAM("MHP_planner_master: Task verification failed: ");
            return false;
        }
    }
    else
    {
        ROS_ERROR_STREAM("MHP_planner_master: No task specified");
        return false;
    }
}

}  // namespace mhp_planner
