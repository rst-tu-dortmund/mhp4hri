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

#ifndef KALMAN_STATE_ESTIMATOR_JOINT_SPACE_H
#define KALMAN_STATE_ESTIMATOR_JOINT_SPACE_H

#include <mhp_robot/robot_obstacle/state_estimators/base_state_estimator_joint_space.h>
#include <Eigen/Dense>
namespace mhp_robot {
namespace robot_obstacle {
namespace state_estimators {

class KalmanStateEstimatorJointSpace : public BaseStateEstimatorJointSpace
{
 public:
    using Ptr  = std::shared_ptr<KalmanStateEstimatorJointSpace>;
    using UPtr = std::unique_ptr<KalmanStateEstimatorJointSpace>;

    KalmanStateEstimatorJointSpace(int id);

    void predict(double t) override;
    void update(const Eigen::Ref<const Eigen::Vector<double, 11>>& angles, double t) override;

    const Eigen::Ref<const Eigen::Matrix<double, 33, 33>> getP() const;
    void setF(const Eigen::Ref<const Eigen::Matrix<double, 33, 33>>& F);
    void setH(const Eigen::Ref<const Eigen::Matrix<double, 11, 33>>& H);
    void setR(const Eigen::Ref<const Eigen::Matrix<double, 11, 11>>& R);
    void setQ(const Eigen::Ref<const Eigen::Matrix<double, 33, 33>>& Q);
    void setInitialP(const Eigen::Ref<const Eigen::Matrix<double, 33, 33>>& P);
    void setInitialState(const Eigen::Ref<const Eigen::Matrix<double, 33, 1>>& state);

 private:
    // 11 joints
    Eigen::Matrix<double, 11, 1> _y;
    Eigen::Matrix<double, 33, 1> _x;
    Eigen::Matrix<double, 33, 33> _P;
    Eigen::Matrix<double, 11, 11> _R;
    Eigen::Matrix<double, 11, 11> _S;
    Eigen::Matrix<double, 33, 11> _K;
    Eigen::Matrix<double, 33, 33> _Q;
    Eigen::Matrix<double, 33, 33> _F;
    Eigen::Matrix<double, 11, 33> _H;
    Eigen::Matrix<double, 33, 33> _I;

    bool _predict = false;
    bool _first   = true;
};

}  // namespace state_estimators
}  // namespace robot_obstacle
}  // namespace mhp_robot
#endif  // KALMAN_STATE_ESTIMATOR_JOINT_SPACE_H
