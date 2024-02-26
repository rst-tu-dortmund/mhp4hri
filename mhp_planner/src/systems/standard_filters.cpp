/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020,
 *  TU Dortmund University, Institute of Control Theory and System Enginnering
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
 *  Authors: Christoph Rösmann
 *  Modifier(s)/Maintainer(s): Maximilian Krämer, Heiko Renz
 *********************************************************************/

#include <mhp_planner/systems/standard_filters.h>

#include <mhp_planner/core/console.h>

#include <Eigen/QR>  // for least-squares computations

#include <numeric>

namespace mhp_planner {

double MovingAverageFilter::filter(double t, double value)
{
    _values.push_front(value);
    while (_values.size() > _window_size) _values.pop_back();

    if (_first_value || _values.empty())
    {
        _first_value = false;
        return value;
    }

    double sum = std::accumulate(_values.begin(), _values.end(), 0.0);
    return sum / (double)(_values.size());
}

void MovingAverageFilter::reset()
{
    _values.clear();
    _first_value = true;
}

bool MovingAverageFilter::fromParameterServer(const std::string& ns)
{
    ros::NodeHandle nh;
    if (!nh.getParam(ns + "/window_size", _window_size))
    {
        PRINT_ERROR("MovingAverageFilter: Could not read window_size from parameter server");
        return false;
    }
    return true;
}

double MovingMedianFilter::filter(double t, double value)
{
    _values.push_front(value);
    while (_values.size() > _window_size) _values.pop_back();

    if (_first_value || _values.empty())
    {
        _first_value = false;
        return value;
    }

    std::deque<double> values_copy = _values;
    std::nth_element(values_copy.begin(), values_copy.begin() + values_copy.size() / 2, values_copy.end());
    return *(values_copy.begin() + values_copy.size() / 2);
}

void MovingMedianFilter::reset()
{
    _values.clear();
    _first_value = true;
}

bool MovingMedianFilter::fromParameterServer(const std::string& ns)
{
    ros::NodeHandle nh;
    if (!nh.getParam(ns + "/window_size", _window_size))
    {
        PRINT_ERROR("MovingAverageFilter: Could not read window_size from parameter server");
        return false;
    }
    return true;
}

double MovingLeastSquaresFilter::filter(double t, double value)
{
    if (_order < 1)
    {
        PRINT_ERROR_NAMED("Polynomial order must be > 0");
        return value;
    }

    if (_num_values == 0)
    {
        allocateMemory();
        _values[0]  = value;
        _time[0]    = t;
        _num_values = 1;
        return value;
    }

    Eigen::VectorXd params;

    // update circular buffers for position and time values
    if (_num_values < _window_size)
    {
        _values[_num_values] = value;
        _time[_num_values]   = t;
        ++_num_values;

        buildRegressionMatrix();

        // perform regression with submatrix
        Eigen::HouseholderQR<Eigen::MatrixXd> qr(_regression_matrix.topRows(_num_values));
        params = qr.solve(_values.head(_num_values));
    }
    else
    {
        _values.head(_window_size - 1) = _values.tail(_window_size - 1);
        _values[_window_size - 1]      = value;
        _time.head(_window_size - 1)   = _time.tail(_window_size - 1);
        _time[_window_size - 1]        = t;

        buildRegressionMatrix();

        // perform regression with full matrix
        Eigen::HouseholderQR<Eigen::MatrixXd> qr(_regression_matrix);
        params = qr.solve(_values);
    }

    assert(params.size() == _order + 1);

    // evaluate polynomial for current time:
    double filtered_value = params[0];  // first coefficient is the bias term
    for (int p = 1; p <= _order; ++p)
    {
        filtered_value += params[p] * std::pow(t, p);
    }

    //    _vel = params[1];  // second coefficient is the bias term for the velocity
    //    for (int p = 2; p <= _order; ++p)
    //    {
    //        _vel += (double)(p)*params[p] * std::pow(t, p - 1);
    //    }
    return filtered_value;
}

void MovingLeastSquaresFilter::reset() { _num_values = 0; }

void MovingLeastSquaresFilter::allocateMemory()
{
    // resize regression_matrix
    _regression_matrix.resize(_window_size, _order + 1);
    // set most left column to 1
    _regression_matrix.leftCols(1).setOnes();

    _values.resize(_window_size);
    _time.resize(_window_size);
}

void MovingLeastSquaresFilter::buildRegressionMatrix()
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

bool MovingLeastSquaresFilter::fromParameterServer(const std::string& ns)
{
    ros::NodeHandle nh;
    if (!nh.getParam(ns + "/window_size", _window_size))
    {
        PRINT_ERROR("MovingAverageFilter: Could not read window_size from parameter server");
        return false;
    }
    if (!nh.getParam(ns + "/polynomial_order", _order))
    {
        PRINT_ERROR("MovingAverageFilter: Could not read polynomial_order from parameter server");
        return false;
    }
    return true;
}

}  // namespace mhp_planner
