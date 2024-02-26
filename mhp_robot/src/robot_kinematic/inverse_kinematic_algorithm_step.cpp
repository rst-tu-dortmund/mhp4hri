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

#include <mhp_robot/robot_kinematic/inverse_kinematic_algorithm_step.h>
#include <mhp_robot/robot_misc/common.h>

namespace mhp_robot {
namespace robot_kinematic {

InverseKinematicAlgorithmStep::InverseKinematicAlgorithmStep(robot_kinematic::RobotKinematic::UPtr robot_kinematic,
                                                             const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& weight_vector,
                                                             double error_length_clamp)
    : _robot_kinematic(std::move(robot_kinematic)), _d_max(error_length_clamp), _weight_matrix(weight_vector.asDiagonal())
{
    _n_dynamic_joints = _robot_kinematic->getRobotUtility().getJointsCount();
    _jacobian         = Eigen::MatrixXd::Zero(6, _n_dynamic_joints);
    _inverse_jacobian = Eigen::MatrixXd::Zero(_n_dynamic_joints, 6);
    _identity_matrix  = Eigen::MatrixXd::Identity(6, 6);
}

void InverseKinematicAlgorithmStep::performIKAStep(const Eigen::Ref<const Eigen::VectorXd>& current_state, Eigen::Ref<Eigen::VectorXd> joint_velocity,
                                                   const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& error_vector, double lambda,
                                                   const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& target_velocities)
{
    _jacobian = _robot_kinematic->getGeometricJacobian(current_state);

    _inverse_jacobian.noalias() = _jacobian.transpose() * (_jacobian * _jacobian.transpose() + lambda * lambda * _identity_matrix).inverse();

    Eigen::Matrix<double, 6, 1> clamped_error_vector = error_vector;
    clampErrorVector(clamped_error_vector);

    joint_velocity.noalias() = _inverse_jacobian * (clamped_error_vector + target_velocities);
}

void InverseKinematicAlgorithmStep::calculateTaskSpaceError(const Eigen::Ref<const Eigen::VectorXd>& state,
                                                            const Eigen::Ref<const Eigen::Vector3d>& target_position,
                                                            const Eigen::Quaterniond& target_orientation,
                                                            Eigen::Ref<Eigen::Matrix<double, 6, 1>> error_vector) const
{
    Eigen::Quaterniond rotated_quaternion;
    const Eigen::Ref<const Eigen::Matrix<double, 4, 4>> current_end_effector_matrix = _robot_kinematic->getEndEffectorMatrix(state);
    Eigen::Quaterniond current_quaternion(current_end_effector_matrix.block<3, 3>(0, 0));

    // pos error
    error_vector.block<3, 1>(0, 0) = target_position - current_end_effector_matrix.block<3, 1>(0, 3);

    // orientation error
    rotated_quaternion = target_orientation * current_quaternion.inverse();

    error_vector.block<3, 1>(3, 0) = RobotCommon::sgn<double>(rotated_quaternion.w()) * rotated_quaternion.vec();
}

void InverseKinematicAlgorithmStep::calculateTaskSpaceErrorScaled(const Eigen::Ref<const Eigen::VectorXd>& state,
                                                                  const Eigen::Ref<const Eigen::Vector3d>& target_position,
                                                                  const Eigen::Quaterniond& target_orientation,
                                                                  Eigen::Ref<Eigen::Matrix<double, 6, 1>> error_vector) const
{
    calculateTaskSpaceError(state, target_position, target_orientation, error_vector);
    scaleErrorVector(error_vector);
}

void InverseKinematicAlgorithmStep::clampErrorVector(Eigen::Ref<Eigen::Matrix<double, 6, 1>> error_vector) const
{
    if (_d_max > 0)
    {
        // ClampMag
        double norm = error_vector.norm();
        if (norm > _d_max) error_vector *= _d_max / norm;
    }
}

void InverseKinematicAlgorithmStep::scaleErrorVector(Eigen::Ref<Eigen::Matrix<double, 6, 1>> error_vector) const
{
    error_vector = _weight_matrix * error_vector;
}

void InverseKinematicAlgorithmStep::setWeights(const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& weight_vector)
{
    _weight_matrix = weight_vector.asDiagonal();
}

void InverseKinematicAlgorithmStep::setLengthClamp(double d_max) { _d_max = d_max; }

double InverseKinematicAlgorithmStep::findOptimalLambda(const Eigen::Ref<const Eigen::VectorXd> initial_state,
                                                        const Eigen::Ref<const Eigen::Vector3d>& target_position,
                                                        const Eigen::Quaterniond& target_orientation,
                                                        const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& target_velocities, double lambda_factor,
                                                        double dt, double lambda)
{
    if (lambda_factor <= 1)
    {
        ROS_ERROR("InverseKinematicAlgorithmStep: lambda_factor must be higher than 1 to determine optimal lambda. Returning provided lambda");
        return lambda;
    }

    Eigen::VectorXd lambda_state     = initial_state;
    Eigen::VectorXd lambda_state_vel = initial_state;
    Eigen::VectorXd initial_error(6);
    Eigen::VectorXd lambda_state_error(6);

    calculateTaskSpaceError(initial_state, target_position, target_orientation, initial_error);
    double init_error = initial_error.norm();

    scaleErrorVector(initial_error);

    performIKAStep(initial_state, lambda_state_vel, initial_error, lambda, target_velocities);
    lambda_state = initial_state + lambda_state_vel * dt;

    calculateTaskSpaceError(lambda_state, target_position, target_orientation, lambda_state_error);
    double lambda_error = lambda_state_error.norm();

    if (lambda <= 1e-6)  // if lambda is neglectible by comparison to 1.0 (lambda < 1e-6) then lambda is not decreased
    {
        if (lambda_error <= init_error)
        {
            return lambda;
        }
        else  // lambda_error > init_error
        {
            for (int i = 0; i < 100; ++i)
            {
                lambda *= lambda_factor;

                performIKAStep(initial_state, lambda_state_vel, initial_error, lambda, target_velocities);
                lambda_state = initial_state + lambda_state_vel * dt;

                calculateTaskSpaceError(lambda_state, target_position, target_orientation, lambda_state_error);

                if (lambda_state_error.norm() <= init_error) return lambda;
            }
        }
    }
    else
    {
        double factor_lambda = lambda / lambda_factor;

        performIKAStep(initial_state, lambda_state_vel, initial_error, factor_lambda, target_velocities);
        lambda_state = initial_state + lambda_state_vel * dt;

        calculateTaskSpaceError(lambda_state, target_position, target_orientation, lambda_state_error);
        double fact_lambda_error = lambda_state_error.norm();

        if (fact_lambda_error <= init_error)
        {
            return factor_lambda;
        }
        else if (fact_lambda_error > init_error && lambda_error <= init_error)
        {
            return lambda;
        }
        else  // lambda_error > init_error && fact_lambda_error > init_error
        {
            for (int i = 0; i < 100; ++i)
            {
                lambda *= lambda_factor;

                performIKAStep(initial_state, lambda_state_vel, initial_error, lambda, target_velocities);
                lambda_state = initial_state + lambda_state_vel * dt;

                calculateTaskSpaceError(lambda_state, target_position, target_orientation, lambda_state_error);

                if (lambda_state_error.norm() <= init_error) return lambda;
            }
        }
    }
    return lambda;  // if after 100 iterations no optimal lambda has been found the latest lambda is returned
}

}  // namespace robot_kinematic
}  // namespace mhp_robot
