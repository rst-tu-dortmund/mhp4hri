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

#ifndef UR_KINEMATIC_H
#define UR_KINEMATIC_H

#include <mhp_robot/robot_kinematic/robot_kinematic.h>
#include <ur_utilities/ur_misc/ur_utility.h>

namespace mhp_robot {
namespace robot_kinematic {

/**
 * @brief The URKinematic class
 *
 * URKimenatic is meant to calculate the kinematics (forward kinematics for all links)
 * for a 6-DOF Universal Robotic Arm.
 *
 * Implementations of forward kinematics (end effector) calculations were borrowed from the ur_moveit package.
 *
 * @see /ur10/ur_moveit/ur_kin.h
 *
 * @author Rodrigo Velasco, Maximilian Krämer
 */

class URKinematic : public RobotKinematic
{
 public:
    using Ptr  = std::shared_ptr<URKinematic>;
    using UPtr = std::unique_ptr<URKinematic>;

    URKinematic();

    RobotKinematic::UPtr createUniqueInstance() const override;
    RobotKinematic::Ptr createSharedInstance() const override;

 private:
    double _d1, _a2, _a3, _d4, _d5, _d6;
    double _c0, _s0;
    double _dx, _dy, _dz;

    void calculateKinematicTransformations(std::vector<Eigen::Matrix<double, 4, 4>>& transformations) override;
    void calculateEndEffectorMatrix(Eigen::Ref<Eigen::Matrix<double, 4, 4>> ee) override;

    void forward(const double* q, double* T);
    void parseLinkLengths();
};
}  // namespace robot_kinematic
}  // namespace mhp_robot

#endif  // UR_KINEMATIC_H
