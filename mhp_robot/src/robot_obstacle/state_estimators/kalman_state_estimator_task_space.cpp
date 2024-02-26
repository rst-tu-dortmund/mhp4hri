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

#include <mhp_robot/robot_obstacle/state_estimators/kalman_state_estimator_task_space.h>

namespace mhp_robot {
namespace robot_obstacle {
namespace state_estimators {

KalmanStateEstimatorTaskSpace::KalmanStateEstimatorTaskSpace(int id) : BaseStateEstimatorTaskSpace(id)
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

void KalmanStateEstimatorTaskSpace::predict(double t)
{
    if (_first) return;

    _x = _F * _x;
    _P = _F * _P * _F.transpose() + _Q;

    _predict = true;
}

void KalmanStateEstimatorTaskSpace::update(const Eigen::Ref<const Eigen::Matrix4d>& pose, double t)
{
    if (!_predict) predict(t);

    if (_first)
    {
        _x.head<3>() = pose.block<3, 1>(0, 3);
        _pose        = pose;
        _first       = false;

        return;
    }

    _y.noalias() = pose.block<3, 1>(0, 3) - _H * _x;
    _S.noalias() = _R + _H * _P * _H.transpose();
    _K.noalias() = _P * _H.transpose() * _S.inverse();

    _x = _x + _K * _y;
    _P = (_I - _K * _H) * _P;  // * (_I - _K * _H).transpose() + _K * _R * _K.transpose();

    // store pose
    _pose                   = pose;
    _pose.block<3, 1>(0, 3) = _x.head<3>();
    _linear_velocity        = _x.segment<3>(3);
    _linear_acceleration    = _x.segment<3>(6);

    _predict = false;
}

void KalmanStateEstimatorTaskSpace::setInitialState(const Eigen::Ref<const Eigen::Matrix<double, 9, 1>>& state)
{
    _x     = state;
    _first = false;
}

void KalmanStateEstimatorTaskSpace::setInitialP(const Eigen::Ref<const Eigen::Matrix<double, 9, 9>>& P) { _P = P; }

void KalmanStateEstimatorTaskSpace::setQ(const Eigen::Ref<const Eigen::Matrix<double, 9, 9>>& Q) { _Q = Q; }

void KalmanStateEstimatorTaskSpace::setR(const Eigen::Ref<const Eigen::Matrix<double, 3, 3>>& R) { _R = R; }

void KalmanStateEstimatorTaskSpace::setH(const Eigen::Ref<const Eigen::Matrix<double, 3, 9>>& H) { _H = H; }

void KalmanStateEstimatorTaskSpace::setF(const Eigen::Ref<const Eigen::Matrix<double, 9, 9>>& F) { _F = F; }

const Eigen::Ref<const Eigen::Matrix<double, 9, 9>> KalmanStateEstimatorTaskSpace::getP() const { return _P; }

}  // namespace state_estimators
}  // namespace robot_obstacle
}  // namespace mhp_robot
