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

#ifndef POLYNOM_STATE_ESTIMATOR_TASK_SPACE_H
#define POLYNOM_STATE_ESTIMATOR_TASK_SPACE_H

#include <mhp_robot/robot_obstacle/state_estimators/base_state_estimator_task_space.h>
namespace mhp_robot {
namespace robot_obstacle {
namespace state_estimators {

class PolynomStateEstimatorTaskSpace : public BaseStateEstimatorTaskSpace
{
 public:
    using Ptr  = std::shared_ptr<PolynomStateEstimatorTaskSpace>;
    using UPtr = std::unique_ptr<PolynomStateEstimatorTaskSpace>;

    PolynomStateEstimatorTaskSpace(int id, int order, int window_size);

    void predict(double t) override;
    void update(const Eigen::Ref<const Eigen::Matrix4d>& pose, double t) override;

    void setF(const Eigen::Ref<const Eigen::Matrix<double, 18, 18>>& F);
    void setInitialState(const Eigen::Ref<const Eigen::Matrix<double, 18, 1>>& state);

 private:
    void allocateMemory();
    void buildRegressionMatrix();
    void extractState();

    // internal states
    Eigen::VectorXd _values_x;
    Eigen::VectorXd _values_y;
    Eigen::VectorXd _values_z;
    Eigen::VectorXd _values_px;
    Eigen::VectorXd _values_py;
    Eigen::VectorXd _values_pz;

    Eigen::VectorXd _time;
    Eigen::MatrixXd _regression_matrix;

    Eigen::Matrix<double, 18, 1> _x;
    Eigen::Matrix<double, 18, 18> _F;

    int _order       = 3;
    int _window_size = 5;
    int _num_values  = 0;
};

}  // namespace state_estimators
}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // POLYNOM_STATE_ESTIMATOR_TASK_SPACE_H
