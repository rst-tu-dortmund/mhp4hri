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
#include <mhp_robot/robot_obstacle/state_estimators/savitzky_golay_state_estimator_joint_space.h>
#include <ros/console.h>
#include <iostream>
namespace mhp_robot {
namespace robot_obstacle {
namespace state_estimators {

SavitzkyGolayStateEstimatorJointSpace::SavitzkyGolayStateEstimatorJointSpace(int id, const std::vector<int>& vel_filter_parameters,
                                                                             const std::vector<int>& acc_filter_parameters)
    : BaseStateEstimatorJointSpace(id)
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
    std::cout << "Filter Velocity SG Estimator: " << _vel_filter.config() << std::endl;
    std::cout << "Filter Acceleration SG Estimator: " << _acc_filter.config() << std::endl;

    _x.setZero();
    _F.setZero();

    allocateMemory();
}

void SavitzkyGolayStateEstimatorJointSpace::predict(double t)
{
    _x = _F * _x;
    extractState();
}

void SavitzkyGolayStateEstimatorJointSpace::update(const Eigen::Ref<const Eigen::Vector<double, 11>>& angles, double t)
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

        _values_q.at(0)  = _values_q0;
        _values_q.at(1)  = _values_q1;
        _values_q.at(2)  = _values_q2;
        _values_q.at(3)  = _values_q3;
        _values_q.at(4)  = _values_q4;
        _values_q.at(5)  = _values_q5;
        _values_q.at(6)  = _values_q6;
        _values_q.at(7)  = _values_q7;
        _values_q.at(8)  = _values_q8;
        _values_q.at(9)  = _values_q9;
        _values_q.at(10) = _values_q10;

        _time[0]    = t;
        _num_values = 1;

        _x.head(11) = angles;
        _angles     = angles;

        _x.segment<22>(11) = Eigen::Vector<double, 22>::Zero();
        return;
    }

    // update circular buffers for position and time values
    if (_num_values < _window_size)
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

        _values_q.at(0)    = _values_q0;
        _values_q.at(1)    = _values_q1;
        _values_q.at(2)    = _values_q2;
        _values_q.at(3)    = _values_q3;
        _values_q.at(4)    = _values_q4;
        _values_q.at(5)    = _values_q5;
        _values_q.at(6)    = _values_q6;
        _values_q.at(7)    = _values_q7;
        _values_q.at(8)    = _values_q8;
        _values_q.at(9)    = _values_q9;
        _values_q.at(10)   = _values_q10;
        _x.segment<22>(11) = Eigen::Vector<double, 22>::Zero();

        _time[_num_values] = t;
        ++_num_values;
    }
    else
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

        _values_q.at(0)  = _values_q0;
        _values_q.at(1)  = _values_q1;
        _values_q.at(2)  = _values_q2;
        _values_q.at(3)  = _values_q3;
        _values_q.at(4)  = _values_q4;
        _values_q.at(5)  = _values_q5;
        _values_q.at(6)  = _values_q6;
        _values_q.at(7)  = _values_q7;
        _values_q.at(8)  = _values_q8;
        _values_q.at(9)  = _values_q9;
        _values_q.at(10) = _values_q10;

        _time.head(_window_size - 1) = _time.tail(_window_size - 1);
        _time[_window_size - 1]      = t;
    }

    // Angles from measurement
    _x.head<11>() = angles;

    // Apply Savitzky-Golay Filter to the Velocity data
    if (_num_values >= 2 * _vel_filter_window_size + 1)
    {
        for (int k = 0; k < 11; ++k)
        {
            std::vector<double> dataVel(2 * _vel_filter_window_size + 1, 0.0);
            for (int j = 0; j < 2 * _vel_filter_window_size + 1; ++j)
            {
                dataVel[j] = (_values_q.at(k)[j]);
            }
            // save filtered value into state vector
            _x(11 + k) = _vel_filter.filter(dataVel) * 25;
        }
    }

    // Apply Savitzky-Golay Filter to the Acceleration data
    if (_num_values >= 2 * _acc_filter_window_size + 1)
    {
        for (int k = 0; k < 11; ++k)
        {
            std::vector<double> dataAcc(2 * _acc_filter_window_size + 1, 0.0);
            for (int j = 0; j < 2 * _acc_filter_window_size + 1; ++j)
            {
                dataAcc[j] = (_values_q.at(k)[j]);
            }
            // save filtered value into state vector
            _x(22 + k) = _acc_filter.filter(dataAcc) * pow(25, 2);
        }
    }

    extractState();
}

void SavitzkyGolayStateEstimatorJointSpace::setF(const Eigen::Ref<const Eigen::Matrix<double, 33, 33>>& F) { _F = F; }

void SavitzkyGolayStateEstimatorJointSpace::setInitialState(const Eigen::Ref<const Eigen::Matrix<double, 33, 1>>& state) { _x = state; }

void SavitzkyGolayStateEstimatorJointSpace::allocateMemory()
{

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

    _values_q.push_back(_values_q0);
    _values_q.push_back(_values_q1);
    _values_q.push_back(_values_q2);
    _values_q.push_back(_values_q3);
    _values_q.push_back(_values_q4);
    _values_q.push_back(_values_q5);
    _values_q.push_back(_values_q6);
    _values_q.push_back(_values_q7);
    _values_q.push_back(_values_q8);
    _values_q.push_back(_values_q9);
    _values_q.push_back(_values_q10);

    _time.resize(_window_size);
}

void SavitzkyGolayStateEstimatorJointSpace::extractState()
{
    // Fill Angles
    _angles = _x.head<11>();

    // Fill dynamic
    _angular_velocity     = _x.segment<11>(11);
    _angular_acceleration = _x.segment<11>(22);
}

void SavitzkyGolayStateEstimatorJointSpace::configureFilter(gram_sg::SavitzkyGolayFilter& filter, const int& filter_window_size,
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