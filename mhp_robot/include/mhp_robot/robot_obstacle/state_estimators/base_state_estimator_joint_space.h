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

#ifndef BASE_STATE_ESTIMATOR_JOINT_SPACE_H
#define BASE_STATE_ESTIMATOR_JOINT_SPACE_H

#include <Eigen/Dense>
#include <memory>

namespace mhp_robot {
namespace robot_obstacle {
namespace state_estimators {

class BaseStateEstimatorJointSpace
{
 public:
    using Ptr  = std::shared_ptr<BaseStateEstimatorJointSpace>;
    using UPtr = std::unique_ptr<BaseStateEstimatorJointSpace>;

    BaseStateEstimatorJointSpace(int id);

    BaseStateEstimatorJointSpace(const BaseStateEstimatorJointSpace&)            = delete;
    BaseStateEstimatorJointSpace(BaseStateEstimatorJointSpace&&)                 = default;
    BaseStateEstimatorJointSpace& operator=(const BaseStateEstimatorJointSpace&) = delete;
    BaseStateEstimatorJointSpace& operator=(BaseStateEstimatorJointSpace&&)      = default;
    virtual ~BaseStateEstimatorJointSpace() {}

    int _id = -1;

    virtual void predict(double t)                                                           = 0;
    virtual void update(const Eigen::Ref<const Eigen::Vector<double, 11>>& angles, double t) = 0;

    void getState(Eigen::Ref<Eigen::Vector<double, 11>> angles, Eigen::Ref<Eigen::Vector<double, 11>> angular_velocity,
                  Eigen::Ref<Eigen::Vector<double, 11>> angular_acceleration) const;

 protected:
    Eigen::Vector<double, 11> _angles;
    Eigen::Vector<double, 11> _angular_velocity;
    Eigen::Vector<double, 11> _angular_acceleration;
};

}  // namespace state_estimators
}  // namespace robot_obstacle
}  // namespace mhp_robot
#endif  // BASE_STATE_ESTIMATOR_JOINT_SPACE_H
