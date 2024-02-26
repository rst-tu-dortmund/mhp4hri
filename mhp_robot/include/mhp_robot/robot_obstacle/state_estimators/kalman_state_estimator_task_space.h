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

#ifndef KALMAN_STATE_ESTIMATOR_TASK_SPACE_H
#define KALMAN_STATE_ESTIMATOR_TASK_SPACE_H

#include <mhp_robot/robot_obstacle/state_estimators/base_state_estimator_task_space.h>

namespace mhp_robot {
namespace robot_obstacle {
namespace state_estimators {

class KalmanStateEstimatorTaskSpace : public BaseStateEstimatorTaskSpace
{
 public:
    using Ptr  = std::shared_ptr<KalmanStateEstimatorTaskSpace>;
    using UPtr = std::unique_ptr<KalmanStateEstimatorTaskSpace>;

    KalmanStateEstimatorTaskSpace(int id);

    void predict(double t) override;
    void update(const Eigen::Ref<const Eigen::Matrix4d>& pose, double t) override;

    const Eigen::Ref<const Eigen::Matrix<double, 9, 9>> getP() const;
    void setF(const Eigen::Ref<const Eigen::Matrix<double, 9, 9>>& F);
    void setH(const Eigen::Ref<const Eigen::Matrix<double, 3, 9>>& H);
    void setR(const Eigen::Ref<const Eigen::Matrix<double, 3, 3>>& R);
    void setQ(const Eigen::Ref<const Eigen::Matrix<double, 9, 9>>& Q);
    void setInitialP(const Eigen::Ref<const Eigen::Matrix<double, 9, 9>>& P);
    void setInitialState(const Eigen::Ref<const Eigen::Matrix<double, 9, 1>>& state);

 private:
    Eigen::Vector3d _y;
    Eigen::Matrix<double, 9, 1> _x;
    Eigen::Matrix<double, 9, 9> _P;
    Eigen::Matrix<double, 3, 3> _R;
    Eigen::Matrix<double, 3, 3> _S;
    Eigen::Matrix<double, 9, 3> _K;
    Eigen::Matrix<double, 9, 9> _Q;
    Eigen::Matrix<double, 9, 9> _F;
    Eigen::Matrix<double, 3, 9> _H;
    Eigen::Matrix<double, 9, 9> _I;

    bool _predict = false;
    bool _first   = true;
};

}  // namespace state_estimators
}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // KALMAN_STATE_ESTIMATOR_TASK_SPACE_H
