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

#ifndef ROBOT_ACCELERATION_CONSTRAINT_H
#define ROBOT_ACCELERATION_CONSTRAINT_H

#include <Eigen/Core>
#include <memory>

namespace mhp_robot {
namespace robot_trajectory_optimization {

class RobotAccelerationConstraint
{
 public:
    using Ptr  = std::shared_ptr<RobotAccelerationConstraint>;
    using UPtr = std::unique_ptr<RobotAccelerationConstraint>;

    RobotAccelerationConstraint(int dimension);

    RobotAccelerationConstraint(const RobotAccelerationConstraint&)            = delete;
    RobotAccelerationConstraint(RobotAccelerationConstraint&&)                 = default;
    RobotAccelerationConstraint& operator=(const RobotAccelerationConstraint&) = delete;
    RobotAccelerationConstraint& operator=(RobotAccelerationConstraint&&)      = default;
    virtual ~RobotAccelerationConstraint() {}

    virtual void compute(const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_prev, double dt,
                         Eigen::Ref<Eigen::VectorXd> cost) const;
    virtual int getDimension() const;

    virtual void setLimits(const Eigen::Ref<const Eigen::ArrayXd>& min, const Eigen::Ref<const Eigen::ArrayXd>& max);
    virtual void getLimits(Eigen::Ref<Eigen::VectorXd> min, Eigen::Ref<Eigen::VectorXd> max) const;

 protected:
    Eigen::ArrayXd _min;
    Eigen::ArrayXd _max;

    bool _active = true;
    int _dim     = 0;
};

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot

#endif  // ROBOT_ACCELERATION_CONSTRAINT_H
