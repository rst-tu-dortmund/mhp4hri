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

#ifndef ROBOT_P_CONTROLLER_JOINT_SPACE_H
#define ROBOT_P_CONTROLLER_JOINT_SPACE_H

#include <Eigen/Core>
#include <memory>

namespace mhp_robot {
namespace robot_trajectory_optimization {

class RobotPControllerJointSpace
{
 public:
    using Ptr  = std::shared_ptr<RobotPControllerJointSpace>;
    using UPtr = std::unique_ptr<RobotPControllerJointSpace>;

    RobotPControllerJointSpace() = default;

    RobotPControllerJointSpace(const RobotPControllerJointSpace&)            = delete;
    RobotPControllerJointSpace(RobotPControllerJointSpace&&)                 = default;
    RobotPControllerJointSpace& operator=(const RobotPControllerJointSpace&) = delete;
    RobotPControllerJointSpace& operator=(RobotPControllerJointSpace&&)      = default;
    virtual ~RobotPControllerJointSpace() {}

    virtual void step(const Eigen::Ref<const Eigen::VectorXd>& x0, const Eigen::Ref<const Eigen::VectorXd>& x_ref,
                      const Eigen::Ref<const Eigen::VectorXd>& u_ref, Eigen::Ref<Eigen::VectorXd> u, bool feed_forward = false);

    virtual void setLimits(const Eigen::Ref<const Eigen::VectorXd>& min, const Eigen::Ref<const Eigen::VectorXd>& max);
    virtual void setWeights(const Eigen::Ref<const Eigen::VectorXd>& weights);

 protected:
    Eigen::VectorXd _weights;
    Eigen::VectorXd _min;
    Eigen::VectorXd _max;
};

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot

#endif  // ROBOT_P_CONTROLLER_JOINT_SPACE_H
