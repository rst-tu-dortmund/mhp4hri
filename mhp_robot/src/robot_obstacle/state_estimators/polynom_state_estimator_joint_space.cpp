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

#include <mhp_robot/robot_obstacle/state_estimators/polynom_state_estimator_joint_space.h>
#include <ros/console.h>
#include <iostream>
namespace mhp_robot {
namespace robot_obstacle {
namespace state_estimators {

PolynomStateEstimatorJointSpace::PolynomStateEstimatorJointSpace(int id, int order, int window_size)
    : BaseStateEstimatorJointSpace(id), _order(order), _window_size(window_size)
{
    ROS_INFO("Polynom fitting order %d", _order);
    ROS_INFO("Polynom window size %d", _window_size);

    _order       = _order < 1 ? 1 : _order;
    _window_size = _window_size < _order + 1 ? _order + 1 : _window_size;

    _x.setZero();
    _F.setZero();

    allocateMemory();
}

void PolynomStateEstimatorJointSpace::predict(double t)
{
    _x = _F * _x;
    extractState();
}

void PolynomStateEstimatorJointSpace::update(const Eigen::Ref<const Eigen::Vector<double, 11>>& angles, double t)
{
    if (_num_values == 0)
    {
        _values_q0[0]  = angles(0);
        _values_q1[0]  = angles(1);
        _values_q2[0]  = angles(2);
        _values_q3[0]  = angles(3);
        _values_q4[0]  = angles(4);
        _values_q5[0]  = angles(5);
        _values_q6[0]  = angles(6);
        _values_q7[0]  = angles(7);
        _values_q8[0]  = angles(8);
        _values_q9[0]  = angles(9);
        _values_q10[0] = angles(10);

        _time[0]    = t;
        _num_values = 1;

        _x.head(11) = angles;
        _angles     = angles;

        return;
    }

    Eigen::VectorXd params_q0, params_q1, params_q2, params_q3, params_q4, params_q5, params_q6, params_q7, params_q8, params_q9, params_q10;

    // update circular buffers for position and time values
    if (_num_values < _window_size)  // if buffer is not full
    {
        _values_q0[_num_values]  = angles(0);
        _values_q1[_num_values]  = angles(1);
        _values_q2[_num_values]  = angles(2);
        _values_q3[_num_values]  = angles(3);
        _values_q4[_num_values]  = angles(4);
        _values_q5[_num_values]  = angles(5);
        _values_q6[_num_values]  = angles(6);
        _values_q7[_num_values]  = angles(7);
        _values_q8[_num_values]  = angles(8);
        _values_q9[_num_values]  = angles(9);
        _values_q10[_num_values] = angles(10);

        _time[_num_values] = t;
        ++_num_values;

        buildRegressionMatrix();

        // perform regression with submatrix
        Eigen::HouseholderQR<Eigen::MatrixXd> qr(_regression_matrix.topRows(_num_values));
        params_q0  = qr.solve(_values_q0.head(_num_values));
        params_q1  = qr.solve(_values_q1.head(_num_values));
        params_q2  = qr.solve(_values_q2.head(_num_values));
        params_q3  = qr.solve(_values_q3.head(_num_values));
        params_q4  = qr.solve(_values_q4.head(_num_values));
        params_q5  = qr.solve(_values_q5.head(_num_values));
        params_q6  = qr.solve(_values_q6.head(_num_values));
        params_q7  = qr.solve(_values_q7.head(_num_values));
        params_q8  = qr.solve(_values_q8.head(_num_values));
        params_q9  = qr.solve(_values_q9.head(_num_values));
        params_q10 = qr.solve(_values_q10.head(_num_values));
    }
    else  // if buffer is full, discard old ones and add new ones
    {
        _values_q0.head(_window_size - 1) = _values_q0.tail(_window_size - 1);
        _values_q0[_window_size - 1]      = angles(0);

        _values_q1.head(_window_size - 1) = _values_q1.tail(_window_size - 1);
        _values_q1[_window_size - 1]      = angles(1);

        _values_q2.head(_window_size - 1) = _values_q2.tail(_window_size - 1);
        _values_q2[_window_size - 1]      = angles(2);

        _values_q3.head(_window_size - 1) = _values_q3.tail(_window_size - 1);
        _values_q3[_window_size - 1]      = angles(3);

        _values_q4.head(_window_size - 1) = _values_q4.tail(_window_size - 1);
        _values_q4[_window_size - 1]      = angles(4);

        _values_q5.head(_window_size - 1) = _values_q5.tail(_window_size - 1);
        _values_q5[_window_size - 1]      = angles(5);

        _values_q6.head(_window_size - 1) = _values_q6.tail(_window_size - 1);
        _values_q6[_window_size - 1]      = angles(6);

        _values_q7.head(_window_size - 1) = _values_q7.tail(_window_size - 1);
        _values_q7[_window_size - 1]      = angles(7);

        _values_q8.head(_window_size - 1) = _values_q8.tail(_window_size - 1);
        _values_q8[_window_size - 1]      = angles(8);

        _values_q9.head(_window_size - 1) = _values_q9.tail(_window_size - 1);
        _values_q9[_window_size - 1]      = angles(9);

        _values_q10.head(_window_size - 1) = _values_q10.tail(_window_size - 1);
        _values_q10[_window_size - 1]      = angles(10);

        _time.head(_window_size - 1) = _time.tail(_window_size - 1);
        _time[_window_size - 1]      = t;

        buildRegressionMatrix();

        // perform regression with full matrix
        Eigen::HouseholderQR<Eigen::MatrixXd> qr(_regression_matrix);
        params_q0  = qr.solve(_values_q0);
        params_q1  = qr.solve(_values_q1);
        params_q2  = qr.solve(_values_q2);
        params_q3  = qr.solve(_values_q3);
        params_q4  = qr.solve(_values_q4);
        params_q5  = qr.solve(_values_q5);
        params_q6  = qr.solve(_values_q6);
        params_q7  = qr.solve(_values_q7);
        params_q8  = qr.solve(_values_q8);
        params_q9  = qr.solve(_values_q9);
        params_q10 = qr.solve(_values_q10);
    }

    assert(params_q0.size() == _order + 1);

    // Pose from measurement
    _x.head<11>() = angles;

    // Linear velocity
    _x(11) = params_q0[1];
    _x(12) = params_q1[1];
    _x(13) = params_q2[1];
    _x(14) = params_q3[1];
    _x(15) = params_q4[1];
    _x(16) = params_q5[1];
    _x(17) = params_q6[1];
    _x(18) = params_q7[1];
    _x(19) = params_q8[1];
    _x(20) = params_q9[1];
    _x(21) = params_q10[1];

    for (int p = 2; p <= _order; ++p)
    {
        _x(11) += p * params_q0[p] * std::pow(t, p - 1);
        _x(12) += p * params_q1[p] * std::pow(t, p - 1);
        _x(13) += p * params_q2[p] * std::pow(t, p - 1);
        _x(14) += p * params_q3[p] * std::pow(t, p - 1);
        _x(15) += p * params_q4[p] * std::pow(t, p - 1);
        _x(16) += p * params_q5[p] * std::pow(t, p - 1);
        _x(17) += p * params_q6[p] * std::pow(t, p - 1);
        _x(18) += p * params_q7[p] * std::pow(t, p - 1);
        _x(19) += p * params_q8[p] * std::pow(t, p - 1);
        _x(20) += p * params_q9[p] * std::pow(t, p - 1);
        _x(21) += p * params_q10[p] * std::pow(t, p - 1);
    }

    // Linear acceleration
    _x(22) = 2 * params_q0[2];
    _x(23) = 2 * params_q1[2];
    _x(24) = 2 * params_q2[2];
    _x(25) = 2 * params_q3[2];
    _x(26) = 2 * params_q4[2];
    _x(27) = 2 * params_q5[2];
    _x(28) = 2 * params_q6[2];
    _x(29) = 2 * params_q7[2];
    _x(30) = 2 * params_q8[2];
    _x(31) = 2 * params_q9[2];
    _x(32) = 2 * params_q10[2];

    for (int p = 3; p <= _order; ++p)
    {
        _x(22) += p * (p - 1) * params_q0[p] * std::pow(t, p - 2);
        _x(23) += p * (p - 1) * params_q1[p] * std::pow(t, p - 2);
        _x(24) += p * (p - 1) * params_q2[p] * std::pow(t, p - 2);
        _x(25) += p * (p - 1) * params_q3[p] * std::pow(t, p - 2);
        _x(26) += p * (p - 1) * params_q4[p] * std::pow(t, p - 2);
        _x(27) += p * (p - 1) * params_q5[p] * std::pow(t, p - 2);
        _x(28) += p * (p - 1) * params_q6[p] * std::pow(t, p - 2);
        _x(29) += p * (p - 1) * params_q7[p] * std::pow(t, p - 2);
        _x(30) += p * (p - 1) * params_q8[p] * std::pow(t, p - 2);
        _x(31) += p * (p - 1) * params_q9[p] * std::pow(t, p - 2);
        _x(32) += p * (p - 1) * params_q10[p] * std::pow(t, p - 2);
    }

    extractState();
}

void PolynomStateEstimatorJointSpace::setF(const Eigen::Ref<const Eigen::Matrix<double, 33, 33>>& F) { _F = F; }

void PolynomStateEstimatorJointSpace::setInitialState(const Eigen::Ref<const Eigen::Matrix<double, 33, 1>>& state) { _x = state; }

void PolynomStateEstimatorJointSpace::allocateMemory()
{
    _regression_matrix.resize(_window_size, _order + 1);

    _regression_matrix.leftCols(1).setOnes();

    _values_q0.resize(_window_size);
    _values_q1.resize(_window_size);
    _values_q2.resize(_window_size);
    _values_q3.resize(_window_size);
    _values_q4.resize(_window_size);
    _values_q5.resize(_window_size);
    _values_q6.resize(_window_size);
    _values_q7.resize(_window_size);
    _values_q8.resize(_window_size);
    _values_q9.resize(_window_size);
    _values_q10.resize(_window_size);

    _time.resize(_window_size);
}

void PolynomStateEstimatorJointSpace::buildRegressionMatrix()
{
    assert(_regression_matrix.rows() == _window_size);
    assert(_regression_matrix.cols() == _order + 1);
    for (int i = 0; i < _num_values; ++i)
    {
        for (int j = 1; j <= _order; ++j)  // skip most left column, since it is already initialized to 1 in allocateMemory()
        {
            _regression_matrix(i, j) = std::pow(_time[i], j);
        }
    }
}

void PolynomStateEstimatorJointSpace::extractState()
{
    // Fill Angles
    _angles = _x.head<11>();

    // Fill dynamic
    _angular_velocity     = _x.segment<11>(11);
    _angular_acceleration = _x.segment<11>(22);
}

}  // namespace state_estimators
}  // namespace robot_obstacle
}  // namespace mhp_robot
