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

#ifndef INVERSE_KINEMATIC_ALGORITHM_STEP_H
#define INVERSE_KINEMATIC_ALGORITHM_STEP_H

#include <mhp_robot/robot_description/robot_description.h>
#include <mhp_robot/robot_kinematic/robot_kinematic.h>
#include <tf_conversions/tf_eigen.h>

namespace mhp_robot {
namespace robot_kinematic {

/**
 * @brief
 *
 * InverseKinematicAlgorithmStep is meant to perform a step of an Inverse Kinematic Algorithm
 * with damped least-squares (DLS) method.
 *
 * @author Rodrigo Velasco
 */

class InverseKinematicAlgorithmStep
{
 public:
    using Ptr  = std::shared_ptr<InverseKinematicAlgorithmStep>;
    using UPtr = std::unique_ptr<InverseKinematicAlgorithmStep>;

    InverseKinematicAlgorithmStep(robot_kinematic::RobotKinematic::UPtr robot_kinematic,
                                  const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& weight_vector, double error_length_clamp);

    InverseKinematicAlgorithmStep(const InverseKinematicAlgorithmStep&)            = delete;
    InverseKinematicAlgorithmStep(InverseKinematicAlgorithmStep&&)                 = default;
    InverseKinematicAlgorithmStep& operator=(const InverseKinematicAlgorithmStep&) = delete;
    InverseKinematicAlgorithmStep& operator=(InverseKinematicAlgorithmStep&&)      = default;
    ~InverseKinematicAlgorithmStep()                                               = default;

    void performIKAStep(const Eigen::Ref<const Eigen::VectorXd>& current_state, Eigen::Ref<Eigen::VectorXd> joint_velocity,
                        const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& error_vector, double lambda,
                        const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& target_velocities = Eigen::Matrix<double, 6, 1>::Zero(6, 1));

    void calculateTaskSpaceError(const Eigen::Ref<const Eigen::VectorXd>& state, const Eigen::Ref<const Eigen::Vector3d>& target_position,
                                 const Eigen::Quaterniond& target_orientation, Eigen::Ref<Eigen::Matrix<double, 6, 1>> error_vector) const;

    void calculateTaskSpaceErrorScaled(const Eigen::Ref<const Eigen::VectorXd>& state, const Eigen::Ref<const Eigen::Vector3d>& target_position,
                                       const Eigen::Quaterniond& target_orientation, Eigen::Ref<Eigen::Matrix<double, 6, 1>> error_vector) const;

    void clampErrorVector(Eigen::Ref<Eigen::Matrix<double, 6, 1>> error_vector) const;
    void scaleErrorVector(Eigen::Ref<Eigen::Matrix<double, 6, 1>> error_vector) const;

    void setWeights(const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& weight_vector);

    void setLengthClamp(double d_max);

    double findOptimalLambda(const Eigen::Ref<const Eigen::VectorXd> initial_state, const Eigen::Ref<const Eigen::Vector3d>& target_position,
                             const Eigen::Quaterniond& target_orientation, const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& target_velocities,
                             double lambda_factor, double dt, double lambda);

 private:
    using RobotCommon = robot_misc::Common;

    RobotKinematic::UPtr _robot_kinematic;

    int _n_dynamic_joints = 0;
    double _d_max         = 0;

    Eigen::MatrixXd _jacobian, _inverse_jacobian, _identity_matrix;
    Eigen::Matrix<double, 6, 6> _weight_matrix;
};

}  // namespace robot_kinematic
}  // namespace mhp_robot

#endif  // INVERSE_KINEMATIC_ALGORITHM_STEP_H
