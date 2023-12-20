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

#ifndef STATE_H
#define STATE_H

#include <Eigen/Dense>
#include <vector>

namespace mhp_robot {
namespace robot_misc {

class State
{
 public:
    enum InterpolationMethod { NONE, LINEAR };

    State();

    const Eigen::Ref<const Eigen::Vector3d> getLinearVelocity(double t = 0, InterpolationMethod interpolation_method = NONE) const;
    const Eigen::Ref<const Eigen::Vector3d> getAngularVelocity(double t = 0, InterpolationMethod interpolation_method = NONE) const;
    const Eigen::Ref<const Eigen::Vector3d> getLinearAcceleration(double t = 0, InterpolationMethod interpolation_method = NONE) const;
    const Eigen::Ref<const Eigen::Vector3d> getAngularAcceleration(double t = 0, InterpolationMethod interpolation_method = NONE) const;
    const Eigen::Ref<const Eigen::Matrix4d> getPose(double t = 0, InterpolationMethod interpolation_method = NONE) const;
    void clear();

    std::vector<double> _times;
    std::vector<Eigen::Vector3d> _linear_velocities;
    std::vector<Eigen::Vector3d> _angular_velocities;
    std::vector<Eigen::Vector3d> _linear_accelerations;
    std::vector<Eigen::Vector3d> _angular_accelerations;
    std::vector<Eigen::Matrix<double, 4, 4>> _poses;

    int getUncertaintyInstance() const { return _uncertainty_instance; }
    int _uncertainty_instance = 0;

 private:
    mutable Eigen::Vector3d _interpolated_linear_velocity;
    mutable Eigen::Vector3d _interpolated_angular_velocity;
    mutable Eigen::Vector3d _interpolated_linear_acceleration;
    mutable Eigen::Vector3d _interpolated_angular_acceleration;
    mutable Eigen::Matrix<double, 4, 4> _interpolated_pose;
    static constexpr double time_zero = 0.001;  // 1ms precision
};
}  // namespace robot_misc
}  // namespace mhp_robot

#endif  // STATE_H
