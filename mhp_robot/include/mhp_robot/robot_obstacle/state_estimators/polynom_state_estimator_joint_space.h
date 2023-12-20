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

#ifndef POLYNOM_STATE_ESTIMATOR_JOINT_SPACE_H
#define POLYNOM_STATE_ESTIMATOR_JOINT_SPACE_H

#include <mhp_robot/robot_obstacle/state_estimators/base_state_estimator_joint_space.h>

namespace mhp_robot {
namespace robot_obstacle {
namespace state_estimators {

class PolynomStateEstimatorJointSpace : public BaseStateEstimatorJointSpace
{
 public:
    using Ptr  = std::shared_ptr<PolynomStateEstimatorJointSpace>;
    using UPtr = std::unique_ptr<PolynomStateEstimatorJointSpace>;

    PolynomStateEstimatorJointSpace(int id, int order, int window_size);

    void predict(double t) override;
    void update(const Eigen::Ref<const Eigen::Vector<double, 11>>& angles, double t) override;

    void setF(const Eigen::Ref<const Eigen::Matrix<double, 33, 33>>& F);
    void setInitialState(const Eigen::Ref<const Eigen::Matrix<double, 33, 1>>& state);

 private:
    void allocateMemory();
    void buildRegressionMatrix();
    void extractState();

    // internal states
    Eigen::VectorXd _values_q0;
    Eigen::VectorXd _values_q1;
    Eigen::VectorXd _values_q2;
    Eigen::VectorXd _values_q3;
    Eigen::VectorXd _values_q4;
    Eigen::VectorXd _values_q5;
    Eigen::VectorXd _values_q6;
    Eigen::VectorXd _values_q7;
    Eigen::VectorXd _values_q8;
    Eigen::VectorXd _values_q9;
    Eigen::VectorXd _values_q10;

    Eigen::VectorXd _time;
    Eigen::MatrixXd _regression_matrix;

    Eigen::Matrix<double, 33, 1> _x;
    Eigen::Matrix<double, 33, 33> _F;

    int _order       = 3;
    int _window_size = 5;
    int _num_values  = 0;
};

}  // namespace state_estimators
}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // POLYNOM_STATE_ESTIMATOR_JOINT_SPACE_H
