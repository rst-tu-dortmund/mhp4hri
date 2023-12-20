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
#include <mhp_planner/tasks/environment.h>
#include <mhp_planner/tasks/task_interface.h>
#include <mhp_robot/SrvStartSimulation.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <memory>
#include <string>

namespace mhp_planner {
class Master
{
 public:
    //! Default constructor
    Master() {}

    void start();

    //! Restore default master settings
    void setDefault();

    //! Load configuration from ROS parameter server (parameters loaded from yaml file in launch file)
    bool loadFromParameterServer(const std::string& namespace_name);

    //! Set master elements
    bool setPlant(const std::string& namespace_name);
    bool setController(const std::string& namespace_name);
    bool setTask(const std::string& namespace_name);

 protected:
    //! Perform the current task and broadcast stream of signals.
    void performTaskCallback(const std_msgs::Bool::ConstPtr& msg);

    // Service callback for only starting the robot
    bool taskCallback(mhp_robot::SrvStartSimulation::Request& request, mhp_robot::SrvStartSimulation::Response& response);

    // Service callback for starting the robot and the environment
    bool startAllCallback(mhp_robot::SrvStartSimulation::Request& request, mhp_robot::SrvStartSimulation::Response& response);

    //! Check if the current configuration seems to be valid (refer to TaskInterface::verify())
    bool verifyConfig();

    //! Execution stop of the current task requested (setting global ok() variable to false, needs to be supported by the selected task)
    void stop()
    {
        setOk(false);  // set global ok flag which should be checked frequently by the current task
    };

 private:
    Environment _environment;
    TaskInterface::Ptr _task;

    ros::NodeHandle _nh;
    ros::Publisher _internal_perform_pub;
    ros::Subscriber _internal_perform_sub;

    ros::ServiceServer _task_service;
    ros::ServiceClient _task_service_client;

    ros::ServiceServer _start_all_service;

    bool _start_task = false;
};

}  // namespace mhp_planner
