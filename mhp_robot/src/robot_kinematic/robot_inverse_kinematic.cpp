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

#include <mhp_robot/robot_kinematic/robot_inverse_kinematic.h>

namespace mhp_robot {
namespace robot_kinematic {

RobotInverseKinematic::RobotInverseKinematic(robot_kinematic::RobotKinematic::UPtr robot_kinematic)
{
    const RobotUtility& robot_utility = robot_kinematic->getRobotUtility();

    // get number of active joints
    _n_dynamic_joints = robot_utility.getJointsCount();

    // get kinematic structure
    _segment_structs = robot_utility.getRobotDescription().getKinematicStructure();

    _initial_state = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(_n_dynamic_joints, 1);
    _solutions     = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(1, _n_dynamic_joints);

    _ika_step = std::make_unique<InverseKinematicAlgorithmStep>(std::move(robot_kinematic), _weight_vector, _d_max);

    _initialized = true;
}

void RobotInverseKinematic::setTaskPoint(const Eigen::Ref<const Eigen::Vector3d>& position, double roll, double pitch, double yaw)
{
    _task_position = position;
    _task_orientation << roll, pitch, yaw;

    calculateIKSolution(_solutions, _n_solutions);
}

void RobotInverseKinematic::setTaskPoint(const Eigen::Ref<const Eigen::Vector3d>& position, const Eigen::Quaterniond& quat_orientation)
{
    _task_position    = position;
    _task_orientation = quat_orientation.toRotationMatrix().eulerAngles(2, 1, 0).reverse();

    calculateIKSolution(_solutions, _n_solutions);
}

void RobotInverseKinematic::setTaskPoint(const Eigen::Ref<const Eigen::Vector3d>& position, const Eigen::Ref<const Eigen::Matrix3d>& rot_orientation)
{
    _task_position    = position;
    _task_orientation = rot_orientation.eulerAngles(2, 1, 0).reverse();

    calculateIKSolution(_solutions, _n_solutions);
}

void RobotInverseKinematic::setTaskPoint(const Eigen::Ref<const Eigen::Vector3d>& position, const Eigen::AngleAxisd& angax_orientation)
{
    _task_position    = position;
    _task_orientation = angax_orientation.toRotationMatrix().eulerAngles(2, 1, 0).reverse();

    calculateIKSolution(_solutions, _n_solutions);
}

int RobotInverseKinematic::getNumSolutions() const { return _n_solutions; }

bool RobotInverseKinematic::isSolutionFeasible() const { return _n_solutions > 0; }

const Eigen::Ref<const Eigen::MatrixXd> RobotInverseKinematic::getSolutions() const { return _solutions; }

void RobotInverseKinematic::setIKAParameters(const Eigen::Ref<const Eigen::VectorXd>& initial_state, int max_iterations, double error_threshold,
                                             double damping_constant, double damping_constant_optimization_factor,
                                             const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& weight_vector, double error_length_clamp)
{
    if (!_ika_step)
    {
        ROS_ERROR("RobotInverseKinematic: IKCA Engine not loaded.");
        return;
    }

    if (initial_state.size() != _initial_state.size())
    {
        ROS_WARN("RobotInverseKinematic: Invalid initial_state size");
        return;
    }
    _initial_state = initial_state;

    if (max_iterations < 1)
    {
        ROS_WARN("RobotInverseKinematic: max_iterations must be higher or equal to 1");
        return;
    }
    _max_iterations = max_iterations;

    if (error_threshold <= 0)
    {
        ROS_WARN("RobotInverseKinematic: error_threshold must be higher than zero");
        return;
    }
    _error_threshold = error_threshold;

    if (damping_constant < 0)
    {
        ROS_WARN("RobotInverseKinematic: damping_constant must be non-negative ");
        return;
    }
    _initial_lambda = damping_constant;

    if (weight_vector.minCoeff() < 0)
    {
        ROS_WARN("RobotInverseKinematic: weights must be non-negative");
        return;
    }
    _weight_vector = weight_vector;

    _d_max         = error_length_clamp;
    _lambda_factor = damping_constant_optimization_factor;

    // update params
    _ika_step->setLengthClamp(_d_max);
    _ika_step->setWeights(_weight_vector);
}

void RobotInverseKinematic::calculateIKSolution(Eigen::MatrixXd& solutions, int& n_solutions)
{
    solutions.resize(1, _n_dynamic_joints);  // IKA only provides one solution
    performIKAIterations(solutions, n_solutions);
}

void RobotInverseKinematic::performIKAIterations(Eigen::Ref<Eigen::MatrixXd> solutions, int& n_solutions)
{
    Eigen::VectorXd error_vector(6);
    Eigen::VectorXd state          = _initial_state;
    Eigen::VectorXd next_state     = _initial_state;
    Eigen::VectorXd joint_velocity = Eigen::VectorXd::Zero(state.rows());
    double lambda                  = _initial_lambda;

    // Initialize error_vector and check if initial position is the disired one.
    Eigen::Quaterniond task_orientation_quat = Eigen::AngleAxisd(_task_orientation(2), Eigen::Vector3d::UnitZ()) *
                                               Eigen::AngleAxisd(_task_orientation(1), Eigen::Vector3d::UnitY()) *
                                               Eigen::AngleAxisd(_task_orientation(0), Eigen::Vector3d::UnitX());

    _ika_step->calculateTaskSpaceError(state, _task_position, task_orientation_quat, error_vector);

    if (error_vector.norm() < _error_threshold)
    {
        n_solutions      = 1;
        solutions.row(0) = state.transpose();
        return;
    }

    for (int i = 0; i < _max_iterations; ++i)
    {
        if (_lambda_factor > 1)
        {
            lambda =
                _ika_step->findOptimalLambda(state, _task_position, task_orientation_quat, Eigen::VectorXd::Zero(6), _lambda_factor, _dt, lambda);
        }

        // update next step
        _ika_step->scaleErrorVector(error_vector);
        _ika_step->performIKAStep(state, joint_velocity, error_vector, lambda);

        next_state += joint_velocity * _dt;

        for (int j = 0; j < _n_dynamic_joints; ++j)
        {
            next_state(j) = std::fmod(next_state(j), 2 * M_PI);

            // angle range: [-pi,pi]
            if (next_state(j) > M_PI) next_state(j) += -2 * M_PI;
            if (next_state(j) < -M_PI) next_state(j) += 2 * M_PI;
        }

        state = next_state;

        // update the error_vector
        _ika_step->calculateTaskSpaceError(state, _task_position, task_orientation_quat, error_vector);

        if (error_vector.norm() < _error_threshold)
        {
            n_solutions      = 1;
            solutions.row(0) = state.transpose();
            return;
        }
    }

    ROS_WARN("RobotInverseKinematic: could not find IK solution within the selected max iterations. Closest state calculated saved as solution");
    n_solutions      = 0;
    solutions.row(0) = state.transpose();
}

}  // namespace robot_kinematic
}  // namespace mhp_robot
