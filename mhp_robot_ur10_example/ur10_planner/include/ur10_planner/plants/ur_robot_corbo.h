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

#ifndef UR_ROBOT_MHP_PLANNER_H
#define UR_ROBOT_MHP_PLANNER_H

#include <mhp_planner/plants/plant_interface.h>
#include <mhp_robot/robot_trajectory_optimization/robot_p_controller_joint_space.h>
#include <ur_utilities/ur_manipulator/ur_manipulator.h>
#include <ur_utilities/ur_misc/ur_utility.h>

namespace mhp_planner {

class URRobot : public PlantInterface
{
 public:
    using Ptr  = std::shared_ptr<URRobot>;
    using UPtr = std::unique_ptr<URRobot>;

    URRobot() = default;

    PlantInterface::Ptr getInstance() const override;

    bool control(const TimeSeries::ConstPtr& u_sequence, const TimeSeries::ConstPtr& x_sequence, const Duration& dt, const Time& t,
                 const std::string& ns = "") override;

    int getInputDimension() const override;
    int getOutputDimension() const override;
    bool requiresFutureControls() const override;
    bool requiresFutureStates() const override;

    bool initialize() override;

    bool output(PlantInterface::OutputVector& output, const Time& t, const std::string& ns = "") override;

    void stop() override;

    void getFutureState(const Time& t, const Eigen::Ref<const Eigen::VectorXd>& x, double duration, Eigen::Ref<Eigen::VectorXd> x_predicted) override;

    bool fromParameterServer(const std::string& ns) override;

 private:
    using RobotPControllerJointSpace = mhp_robot::robot_trajectory_optimization::RobotPControllerJointSpace;
    using URManipulator              = mhp_robot::robot_manipulator::URManipulator;
    using URUtility                  = mhp_robot::robot_misc::URUtility;

    enum class ControlMode { OPEN_LOOP, CLOSED_LOOP };

    URManipulator _ur_manipulator;
    RobotPControllerJointSpace::UPtr _tracker;

    ControlMode _control_mode = ControlMode::OPEN_LOOP;

    TimeSeries _states;
    TimeSeries _controls;

    // open-loop
    int _num_controls = 1;

    // closed-loop
    Eigen::VectorXd _weights;
    Eigen::VectorXd _u_min;
    Eigen::VectorXd _u_max;

    bool _feedforward     = true;
    bool _real_robot      = false;
    bool _reset_on_start  = true;
    bool _initialized     = false;
    bool _ignore_controls = false;
};

FACTORY_REGISTER_PLANT(URRobot)

}  // namespace mhp_planner

#endif  // UR_ROBOT_MHP_PLANNER_H
