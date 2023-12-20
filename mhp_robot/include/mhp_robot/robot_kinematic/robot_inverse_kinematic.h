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
 *  Authors: Rodrigo Velasco, Maximilian Krämer
 *  Maintainer(s)/Modifier(s): Heiko Renz
 *********************************************************************/

#ifndef ROBOT_INVERSE_KINEMATIC_H
#define ROBOT_INVERSE_KINEMATIC_H

#include <mhp_robot/robot_kinematic/inverse_kinematic_algorithm_step.h>
#include <mhp_robot/robot_kinematic/robot_kinematic.h>
#include <mhp_robot/robot_misc/common.h>

namespace mhp_robot {
namespace robot_kinematic {

/**
 * @brief
 *
 * RobotInvKimenatic is meant to calculate inverse kinematic solution(s)
 * for a robot defined by a RobotDescription structure.
 *
 * @author Rodrigo Velasco, Maximilian Krämer
 */

class RobotInverseKinematic
{
 public:
    using Ptr  = std::shared_ptr<RobotInverseKinematic>;
    using UPtr = std::unique_ptr<RobotInverseKinematic>;

    RobotInverseKinematic(RobotKinematic::UPtr robot_kinematic);

    RobotInverseKinematic(const RobotInverseKinematic&)            = delete;
    RobotInverseKinematic(RobotInverseKinematic&&)                 = default;
    RobotInverseKinematic& operator=(const RobotInverseKinematic&) = delete;
    RobotInverseKinematic& operator=(RobotInverseKinematic&&)      = default;
    virtual ~RobotInverseKinematic() {}

    // Set the task space target meant to be solved.
    // @param position            Robot's end effector cartesian position (x,y,z)
    // @param roll,pitch,yaw      Robot's end effector orientation
    void setTaskPoint(const Eigen::Ref<const Eigen::Vector3d>& position, double roll, double pitch, double yaw);

    // Set the task space target meant to be solved.
    // @param position            Robot's end effector cartesian position (x,y,z)
    // @param quat_orientation    Robot's end effector orientation (quaternion)
    void setTaskPoint(const Eigen::Ref<const Eigen::Vector3d>& position, const Eigen::Quaterniond& quat_orientation);

    // Set the task space target meant to be solved.
    // @param position            Robot's end effector cartesian position (x,y,z)
    // @param rot_orientation     Robot's end effector orientation (rotation matrix)
    void setTaskPoint(const Eigen::Ref<const Eigen::Vector3d>& position, const Eigen::Ref<const Eigen::Matrix3d>& rot_orientation);

    // Set the task space target meant to be solved.
    // @param position            Robot's end effector cartesian position (x,y,z)
    // @param angax_orientation   Robot's end effector orientation (angle axis)
    void setTaskPoint(const Eigen::Ref<const Eigen::Vector3d>& position, const Eigen::AngleAxisd& angax_orientation);

    void setIKAParameters(const Eigen::Ref<const Eigen::VectorXd>& initial_state, int max_iterations, double error_threshold, double damping_constant,
                          double damping_constant_optimization_factor, const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& weight_vector,
                          double error_length_clamp);

    int getNumSolutions() const;

    const Eigen::Ref<const Eigen::MatrixXd> getSolutions() const;

    bool isSolutionFeasible() const;

 protected:
    using RobotUtility = robot_misc::RobotUtility;

    std::vector<robot_misc::Segment> _segment_structs;

    Eigen::Vector3d _task_orientation;
    Eigen::Vector3d _task_position;

    int _n_dynamic_joints = 0;
    bool _initialized     = false;

    virtual void calculateIKSolution(Eigen::MatrixXd& solutions, int& n_solutions);  // cannot use Eigen::Ref here since solutions gets resized

 private:
    InverseKinematicAlgorithmStep::UPtr _ika_step;

    Eigen::Matrix<double, Eigen::Dynamic, 1> _initial_state;
    Eigen::Matrix<double, 6, 1> _weight_vector;

    Eigen::MatrixXd _solutions;
    int _n_solutions = 0;

    int _max_iterations     = 1000;
    double _error_threshold = 1e-6;
    double _initial_lambda  = 0.01;
    double _lambda_factor   = 10;
    double _d_max           = 0.2;
    bool _clamped_method    = true;
    double _dt              = 1;

    void performIKAIterations(Eigen::Ref<Eigen::MatrixXd> solutions, int& n_solutions);
};

}  // namespace robot_kinematic
}  // namespace mhp_robot

#endif  // ROBOT_INVERSE_KINEMATIC_H
