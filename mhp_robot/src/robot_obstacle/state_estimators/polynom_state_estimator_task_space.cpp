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

#include <mhp_robot/robot_obstacle/state_estimators/polynom_state_estimator_task_space.h>

namespace mhp_robot {
namespace robot_obstacle {
namespace state_estimators {

PolynomStateEstimatorTaskSpace::PolynomStateEstimatorTaskSpace(int id, int order, int window_size)
    : BaseStateEstimatorTaskSpace(id), _order(order), _window_size(window_size)
{
    _order       = _order < 1 ? 1 : _order;
    _window_size = _window_size < _order + 1 ? _order + 1 : _window_size;

    _x.setZero();
    _F.setZero();

    allocateMemory();
}

void PolynomStateEstimatorTaskSpace::predict(double t)
{
    _x = _F * _x;
    extractState();
}

void PolynomStateEstimatorTaskSpace::update(const Eigen::Ref<const Eigen::Matrix4d>& pose, double t)
{
    if (_num_values == 0)
    {
        _values_x[0] = pose(0, 3);
        _values_y[0] = pose(1, 3);
        _values_z[0] = pose(2, 3);

        _time[0]    = t;
        _num_values = 1;

        _x.head(3) = pose.block<3, 1>(0, 3);
        _pose      = pose;

        return;
    }
    Eigen::AngleAxisd aa(pose.block<3, 3>(0, 0));
    Eigen::Vector3d euler_vector = aa.axis() * aa.angle();

    Eigen::VectorXd params_x, params_y, params_z, params_px, params_py, params_pz;

    // update circular buffers for position and time values
    if (_num_values < _window_size)
    {
        _values_x[_num_values] = pose(0, 3);
        _values_y[_num_values] = pose(1, 3);
        _values_z[_num_values] = pose(2, 3);

        _values_px[_num_values] = euler_vector(0);
        _values_py[_num_values] = euler_vector(1);
        _values_pz[_num_values] = euler_vector(2);

        _time[_num_values] = t;
        ++_num_values;

        buildRegressionMatrix();

        // perform regression with submatrix
        Eigen::HouseholderQR<Eigen::MatrixXd> qr(_regression_matrix.topRows(_num_values));
        params_x  = qr.solve(_values_x.head(_num_values));
        params_y  = qr.solve(_values_y.head(_num_values));
        params_z  = qr.solve(_values_z.head(_num_values));
        params_px = qr.solve(_values_px.head(_num_values));
        params_py = qr.solve(_values_py.head(_num_values));
        params_pz = qr.solve(_values_pz.head(_num_values));
    }
    else
    {
        _values_x.head(_window_size - 1) = _values_x.tail(_window_size - 1);
        _values_x[_window_size - 1]      = pose(0, 3);

        _values_y.head(_window_size - 1) = _values_y.tail(_window_size - 1);
        _values_y[_window_size - 1]      = pose(1, 3);

        _values_z.head(_window_size - 1) = _values_z.tail(_window_size - 1);
        _values_z[_window_size - 1]      = pose(2, 3);

        _values_px.head(_window_size - 1) = _values_px.tail(_window_size - 1);
        _values_px[_window_size - 1]      = euler_vector(0);

        _values_py.head(_window_size - 1) = _values_py.tail(_window_size - 1);
        _values_py[_window_size - 1]      = euler_vector(1);

        _values_pz.head(_window_size - 1) = _values_pz.tail(_window_size - 1);
        _values_pz[_window_size - 1]      = euler_vector(2);

        _time.head(_window_size - 1) = _time.tail(_window_size - 1);
        _time[_window_size - 1]      = t;

        buildRegressionMatrix();

        // perform regression with full matrix
        Eigen::HouseholderQR<Eigen::MatrixXd> qr(_regression_matrix);
        params_x  = qr.solve(_values_x);
        params_y  = qr.solve(_values_y);
        params_z  = qr.solve(_values_z);
        params_px = qr.solve(_values_px);
        params_py = qr.solve(_values_py);
        params_pz = qr.solve(_values_pz);
    }

    assert(params_x.size() == _order + 1);

    // Pose from measurement
    _x.head<3>()     = pose.block<3, 1>(0, 3);
    _x.segment<3>(9) = euler_vector;

    // Linear velocity
    _x(3) = params_x[1];
    _x(4) = params_y[1];
    _x(5) = params_z[1];

    for (int p = 2; p <= _order; ++p)
    {
        _x(3) += p * params_x[p] * std::pow(t, p - 1);
        _x(4) += p * params_y[p] * std::pow(t, p - 1);
        _x(5) += p * params_z[p] * std::pow(t, p - 1);
    }

    // Linear acceleration
    _x(6) = 2 * params_x[2];
    _x(7) = 2 * params_y[2];
    _x(8) = 2 * params_z[2];

    for (int p = 3; p <= _order; ++p)
    {
        _x(6) += p * (p - 1) * params_x[p] * std::pow(t, p - 2);
        _x(7) += p * (p - 1) * params_y[p] * std::pow(t, p - 2);
        _x(8) += p * (p - 1) * params_z[p] * std::pow(t, p - 2);
    }

    // Angular velocity
    _x(12) = params_px[1];
    _x(13) = params_py[1];
    _x(14) = params_pz[1];

    for (int p = 2; p <= _order; ++p)
    {
        _x(12) += p * params_px[p] * std::pow(t, p - 1);
        _x(13) += p * params_py[p] * std::pow(t, p - 1);
        _x(14) += p * params_pz[p] * std::pow(t, p - 1);
    }

    // Linear acceleration
    _x(15) = 2 * params_px[2];
    _x(16) = 2 * params_py[2];
    _x(17) = 2 * params_pz[2];

    for (int p = 3; p <= _order; ++p)
    {
        _x(15) += p * (p - 1) * params_px[p] * std::pow(t, p - 2);
        _x(16) += p * (p - 1) * params_py[p] * std::pow(t, p - 2);
        _x(17) += p * (p - 1) * params_pz[p] * std::pow(t, p - 2);
    }

    extractState();
}

void PolynomStateEstimatorTaskSpace::setF(const Eigen::Ref<const Eigen::Matrix<double, 18, 18>>& F) { _F = F; }

void PolynomStateEstimatorTaskSpace::setInitialState(const Eigen::Ref<const Eigen::Matrix<double, 18, 1>>& state) { _x = state; }

void PolynomStateEstimatorTaskSpace::allocateMemory()
{
    _regression_matrix.resize(_window_size, _order + 1);

    _regression_matrix.leftCols(1).setOnes();

    _values_x.resize(_window_size);
    _values_y.resize(_window_size);
    _values_z.resize(_window_size);

    _values_px.resize(_window_size);
    _values_py.resize(_window_size);
    _values_pz.resize(_window_size);

    _time.resize(_window_size);
}

void PolynomStateEstimatorTaskSpace::buildRegressionMatrix()
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

void PolynomStateEstimatorTaskSpace::extractState()
{
    // AngAx to rotation matrix
    Eigen::AngleAxisd orientation(_x.segment<3>(9).norm(), _x.segment<3>(9).normalized());

    // Fill Pose
    _pose.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    _pose.block<3, 1>(0, 3) = _x.head<3>();

    // Fill linear dynamic
    _linear_velocity     = _x.segment<3>(3);
    _linear_acceleration = _x.segment<3>(6);

    // Fill angular dynamic
    _angular_velocity     = _x.segment<3>(12);
    _angular_acceleration = _x.segment<3>(15);
}

}  // namespace state_estimators
}  // namespace robot_obstacle
}  // namespace mhp_robot
