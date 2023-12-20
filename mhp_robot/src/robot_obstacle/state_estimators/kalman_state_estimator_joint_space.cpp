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

#include <mhp_robot/robot_obstacle/state_estimators/kalman_state_estimator_joint_space.h>
#include <iostream>
namespace mhp_robot {
namespace robot_obstacle {
namespace state_estimators {

KalmanStateEstimatorJointSpace::KalmanStateEstimatorJointSpace(int id) : BaseStateEstimatorJointSpace(id)
{
    _x.setZero();
    _P.setZero();
    _R.setZero();
    _S.setZero();
    _Q.setZero();
    _K.setZero();
    _F.setZero();
    _H.setZero();
    _I.setIdentity();
}

void KalmanStateEstimatorJointSpace::predict(double t)
{
    if (_first) return;

    _x = _F * _x;
    _P = _F * _P * _F.transpose() + _Q;

    _predict = true;
}

void KalmanStateEstimatorJointSpace::update(const Eigen::Ref<const Eigen::Vector<double, 11>>& angles, double t)
{
    if (!_predict) predict(t);
    if (_first)
    {
        _x.head<11>() = angles;
        _first        = false;

        return;
    }

    _y.noalias() = angles - _H * _x;
    _S.noalias() = _R + _H * _P * _H.transpose();
    _K.noalias() = _P * _H.transpose() * _S.inverse();

    _x = _x + _K * _y;
    _P = (_I - _K * _H) * _P;  // * (_I - _K * _H).transpose() + _K * _R * _K.transpose();

    // store angles
    _angles               = _x.head<11>();
    _angular_velocity     = _x.segment<11>(11);
    _angular_acceleration = _x.segment<11>(22);

    _predict = false;
}

void KalmanStateEstimatorJointSpace::setInitialState(const Eigen::Ref<const Eigen::Matrix<double, 33, 1>>& state)
{
    _x     = state;
    _first = false;
}

void KalmanStateEstimatorJointSpace::setInitialP(const Eigen::Ref<const Eigen::Matrix<double, 33, 33>>& P) { _P = P; }

void KalmanStateEstimatorJointSpace::setQ(const Eigen::Ref<const Eigen::Matrix<double, 33, 33>>& Q) { _Q = Q; }

void KalmanStateEstimatorJointSpace::setR(const Eigen::Ref<const Eigen::Matrix<double, 11, 11>>& R) { _R = R; }

void KalmanStateEstimatorJointSpace::setH(const Eigen::Ref<const Eigen::Matrix<double, 11, 33>>& H) { _H = H; }

void KalmanStateEstimatorJointSpace::setF(const Eigen::Ref<const Eigen::Matrix<double, 33, 33>>& F) { _F = F; }

const Eigen::Ref<const Eigen::Matrix<double, 33, 33>> KalmanStateEstimatorJointSpace::getP() const { return _P; }

}  // namespace state_estimators
}  // namespace robot_obstacle
}  // namespace mhp_robot
