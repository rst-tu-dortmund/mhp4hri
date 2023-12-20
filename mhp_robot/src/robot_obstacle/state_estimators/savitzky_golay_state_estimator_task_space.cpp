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

#ifdef SG

#include <mhp_robot/robot_obstacle/state_estimators/savitzky_golay_state_estimator_task_space.h>
#include <iostream>

namespace mhp_robot {
namespace robot_obstacle {
namespace state_estimators {

SavitzkyGolayStateEstimatorTaskSpace::SavitzkyGolayStateEstimatorTaskSpace(int id, const std::vector<int>& vel_filter_parameters,
                                                                           const std::vector<int>& acc_filter_parameters)
    : BaseStateEstimatorTaskSpace(id)
{

    _vel_filter_window_size      = vel_filter_parameters[0];
    _vel_filter_init_point       = vel_filter_parameters[0];
    _vel_filter_order            = vel_filter_parameters[1];
    _vel_filter_derivation_order = 1;

    _acc_filter_window_size      = acc_filter_parameters[0];
    _acc_filter_init_point       = acc_filter_parameters[0];
    _acc_filter_order            = acc_filter_parameters[1];
    _acc_filter_derivation_order = 2;

    if (_vel_filter_window_size >= _acc_filter_window_size)
    {
        _window_size = _vel_filter_window_size * 2 + 1;
    }
    else
    {
        _window_size = _acc_filter_window_size * 2 + 1;
    }

    // Configure the filter for velocity and acceleration
    configureFilter(_vel_filter, _vel_filter_window_size, _vel_filter_order, _vel_filter_init_point, _vel_filter_derivation_order);
    configureFilter(_acc_filter, _acc_filter_window_size, _acc_filter_order, _acc_filter_init_point, _acc_filter_derivation_order);

    // Output Filter Configurations
    std::cout << "Filter Velocity SG Estimator Task Space: " << _vel_filter.config() << std::endl;
    std::cout << "Filter Acceleration SG Estimator Task Space: " << _acc_filter.config() << std::endl;

    _x.setZero();
    _F.setZero();

    allocateMemory();
}

void SavitzkyGolayStateEstimatorTaskSpace::predict(double t)
{
    _x = _F * _x;
    extractState();
}

void SavitzkyGolayStateEstimatorTaskSpace::update(const Eigen::Ref<const Eigen::Matrix4d>& pose, double t)
{
    if (_num_values == 0)
    {
        _values_x[0] = pose(0, 3);
        _values_y[0] = pose(1, 3);
        _values_z[0] = pose(2, 3);

        _values_px[0] = 0;
        _values_py[0] = 0;
        _values_pz[0] = 0;

        _time[0]    = t;
        _num_values = 1;

        _x.head(3) = pose.block<3, 1>(0, 3);
        _pose      = pose;

        _x.segment<15>(3) = Eigen::Vector<double, 15>::Zero();

        _values_pos.at(0) = _values_x;
        _values_pos.at(1) = _values_y;
        _values_pos.at(2) = _values_z;

        _values_rot.at(0) = _values_px;
        _values_rot.at(1) = _values_py;
        _values_rot.at(2) = _values_pz;

        return;
    }
    Eigen::AngleAxisd aa(pose.block<3, 3>(0, 0));
    Eigen::Vector3d euler_vector = aa.axis() * aa.angle();

    // update circular buffers for position and time values
    if (_num_values < _window_size)
    {
        _values_x[_num_values] = pose(0, 3);
        _values_y[_num_values] = pose(1, 3);
        _values_z[_num_values] = pose(2, 3);

        _values_px[_num_values] = euler_vector(0);
        _values_py[_num_values] = euler_vector(1);
        _values_pz[_num_values] = euler_vector(2);

        _x = Eigen::Vector<double, 18>::Zero();

        _values_pos.at(0) = _values_x;
        _values_pos.at(1) = _values_y;
        _values_pos.at(2) = _values_z;

        _values_rot.at(0) = _values_px;
        _values_rot.at(1) = _values_py;
        _values_rot.at(2) = _values_pz;

        _time[_num_values] = t;
        ++_num_values;
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

        _values_pos.at(0) = _values_x;
        _values_pos.at(1) = _values_y;
        _values_pos.at(2) = _values_z;

        _values_rot.at(0) = _values_px;
        _values_rot.at(1) = _values_py;
        _values_rot.at(2) = _values_pz;
    }

    // Pose from measurement
    _x.head<3>()     = pose.block<3, 1>(0, 3);
    _x.segment<3>(9) = euler_vector;

    // Apply Savitzky-Golay Filter for the Velocity data
    if (_num_values >= 2 * _vel_filter_window_size + 1)
    {
        for (int k = 0; k < 3; ++k)
        {
            std::vector<double> dataPos(2 * _vel_filter_window_size + 1, 0.0);
            std::vector<double> dataRot(2 * _vel_filter_window_size + 1, 0.0);

            for (int j = 0; j < 2 * _vel_filter_window_size + 1; ++j)
            {
                dataPos[j] = (_values_pos.at(k)[j]);
                dataRot[j] = (_values_rot.at(k)[j]);
            }
            // save filtered value into state vector
            _x(3 + k)  = _vel_filter.filter(dataPos) * 25;
            _x(12 + k) = _vel_filter.filter(dataRot) * 25;
        }
    }

    // Apply Savitzky-Golay Filter for the Acceleration data
    if (_num_values >= 2 * _acc_filter_window_size + 1)
    {
        for (int k = 0; k < 3; ++k)
        {
            std::vector<double> dataPosAcc(2 * _acc_filter_window_size + 1, 0.0);
            std::vector<double> dataRotAcc(2 * _acc_filter_window_size + 1, 0.0);
            for (int j = 0; j < 2 * _acc_filter_window_size + 1; ++j)
            {
                dataPosAcc[j] = (_values_pos.at(k)[j]);
                dataRotAcc[j] = (_values_rot.at(k)[j]);
            }
            // save filtered value into state vector
            _x(6 + k)  = _acc_filter.filter(dataPosAcc) * pow(25, 2);
            _x(15 + k) = _acc_filter.filter(dataRotAcc) * pow(25, 2);
        }
    }

    extractState();
}

void SavitzkyGolayStateEstimatorTaskSpace::setF(const Eigen::Ref<const Eigen::Matrix<double, 18, 18>>& F) { _F = F; }

void SavitzkyGolayStateEstimatorTaskSpace::setInitialState(const Eigen::Ref<const Eigen::Matrix<double, 18, 1>>& state) { _x = state; }

void SavitzkyGolayStateEstimatorTaskSpace::allocateMemory()
{
    _values_x.resize(_window_size);
    _values_y.resize(_window_size);
    _values_z.resize(_window_size);

    _values_px.resize(_window_size);
    _values_py.resize(_window_size);
    _values_pz.resize(_window_size);

    _values_pos.push_back(_values_x);
    _values_pos.push_back(_values_y);
    _values_pos.push_back(_values_z);

    _values_rot.push_back(_values_px);
    _values_rot.push_back(_values_py);
    _values_rot.push_back(_values_pz);

    _time.resize(_window_size);
}

void SavitzkyGolayStateEstimatorTaskSpace::extractState()
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

void SavitzkyGolayStateEstimatorTaskSpace::configureFilter(gram_sg::SavitzkyGolayFilter& filter, const int& filter_window_size,
                                                           const int& filter_order, const int& filter_init_point, const int& filter_derivation_order)
{
    // Define Filter Configuration with the given arguments
    gram_sg::SavitzkyGolayFilterConfig filterConfig(filter_window_size, filter_init_point, filter_order, filter_derivation_order);
    // reconfigure the class member filter given in the arguments
    filter.configure(filterConfig);
}

}  // namespace state_estimators
}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // SG