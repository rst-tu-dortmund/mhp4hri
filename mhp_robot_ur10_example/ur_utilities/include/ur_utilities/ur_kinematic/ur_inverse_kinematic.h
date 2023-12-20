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
 *  Authors: Rodrigo Velasco
 *  Maintainer(s)/Modifier(s): Maximilian Krämer, Heiko Renz
 *
 * Notes: Inverse kinematic functions were originally authored by
 * Kelsey Hawkins (kphawkins@gatech.edu).
 * Copyright (c) 2013, Georgia Institute of Technology
 *********************************************************************/

#ifndef UR_INVERSE_KINEMATIC_H
#define UR_INVERSE_KINEMATIC_H

#include <mhp_robot/robot_kinematic/robot_inverse_kinematic.h>
#include <mhp_robot/robot_misc/common.h>
#include <ur_utilities/ur_kinematic/ur_kinematic.h>

namespace mhp_robot {
namespace robot_kinematic {

/**
 * @brief The URInverseKinematic class
 *
 * URInvKimenatic is meant to calculate multiple inverse kinematic solutions
 * for a 6-DOF Universal Robotic Arm from the /world frame to the /ee_link frame.
 *
 * Implementations of inverse kinematics calculations were borrowed from the ur_moveit package.
 *
 * @see /ur10/ur_moveit/ur_kin.h
 *
 * @author Rodrigo Velasco, Maximilian Krämer
 */

class URInverseKinematic : public RobotInverseKinematic
{
 public:
    using Ptr  = std::shared_ptr<URInverseKinematic>;
    using UPtr = std::unique_ptr<URInverseKinematic>;

    URInverseKinematic();

    void setDesiredLastJoint(double value);

 private:
    double _q6_desired = 0;
    double _d1, _a2, _a3, _d4, _d5, _d6;

    Eigen::Matrix<double, 9, 1> _links;

    void calculateIKSolution(Eigen::MatrixXd& solutions, int& n_solutions) override;

    void defineTransformation(const Eigen::Ref<const Eigen::Vector3d>& pos, const Eigen::Ref<const Eigen::Vector3d>& rpy, double* T);
    int inverse(const double* T, double* q_sols);
    void parseLinkLengths();
};

}  // namespace robot_kinematic
}  // namespace mhp_robot

#endif  // UR_INVERSE_KINEMATIC_H
