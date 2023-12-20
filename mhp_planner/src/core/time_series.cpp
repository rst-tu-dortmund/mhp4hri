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

#include <mhp_planner/core/console.h>
#include <mhp_planner/core/time_series.h>

#include <cmath>
#include <iomanip>

namespace mhp_planner {

void TimeSeries::setValueDimension(int value_dim)
{
    if (value_dim != _value_dim)
    {
        clear();
        _value_dim = value_dim;
    }
}

void TimeSeries::reserve(int time_dim, int value_dim)
{
    _time.reserve(time_dim);
    _values.reserve(time_dim * value_dim);
}

bool TimeSeries::add(double time, const std::vector<double>& values)
{
    if (values.empty()) return true;
    if (_time.empty())
        _value_dim = values.size();  // if this is the first value, inherit value dimension from input
    else if (values.size() != _value_dim)
    {
        PRINT_ERROR("TimeSeries::add(): dimension mismatch: values.size() must be " << _value_dim);
        return false;
    }
    _time.push_back(time);
    // just concatenate vectors (since we are storing in column-major)
    _values.insert(_values.end(), values.begin(), values.end());
    return true;
}

bool TimeSeries::add(double time, const Eigen::Ref<const Eigen::VectorXd>& values)
{
    if (_time.empty())
        _value_dim = values.size();  // if this is the first value, inherit value dimension from input
    else if (values.size() != _value_dim)
    {
        PRINT_ERROR("TimeSeries::add(): dimension mismatch: values.size() must be " << _value_dim);
        return false;
    }
    _time.push_back(time);
    // just concatenate vectors (since we are storing in column-major)
    _values.insert(_values.end(), values.data(), values.data() + _value_dim);
    return true;
}

bool TimeSeries::set(const Eigen::Ref<const Eigen::VectorXd>& time, const Eigen::Ref<const Eigen::MatrixXd>& values_matrix, double time_from_start)
{
    if (values_matrix.cols() != time.size())
    {
        PRINT_ERROR("TimeSeries::set(): time.size() != values_matrix.cols()");
        clear();
        return false;
    }
    _time_from_start = time_from_start;

    _value_dim = values_matrix.rows();
    _time.assign(time.data(), time.data() + time.size());
    if (values_matrix.IsRowMajor)
    {
        // TODO(roesmann) here is place for optimization: avoid default construction in resize:
        _values.resize(_value_dim * getTimeDimension());
        ValuesMatMap target(_values.data(), _value_dim, getTimeDimension());
        target = values_matrix;
    }
    else
    {
        const int size = values_matrix.rows() * values_matrix.cols();
        _values.assign(values_matrix.data(), values_matrix.data() + size);
    }
    return true;
}

bool TimeSeries::set(const std::vector<double>& time, const std::vector<Eigen::VectorXd>& values_vector, double time_from_start)
{
    if (values_vector.size() != time.size())
    {
        PRINT_ERROR_NAMED("time.size() != values_vector.size()");
        clear();
        return false;
    }

    _time_from_start = time_from_start;

    if (time.empty())
    {
        clear();
        return true;  // TODO(roesmann) or better false?
    }

    _value_dim = values_vector.front().size();

    reserve(time.size(), _value_dim);

    _time = time;

    for (const Eigen::VectorXd& vec : values_vector)
    {
        assert(vec.size() == _value_dim);
        _values.insert(_values.end(), vec.data(), vec.data() + vec.size());
    }

    if (_value_dim * values_vector.size() != _values.size())
    {
        PRINT_ERROR_NAMED("Vectors in values_vector must be of equal size. Clearing time series object.");
        clear();
        return false;
    }
    return true;
}

bool TimeSeries::insert(const TimeSeries::ConstPtr time_series, bool overwrite)
{
    if (!time_series) return false;

    if (time_series->isEmpty())
    {
        return true;
    }

    if (isEmpty())
    {
        _time            = time_series->_time;
        _values          = time_series->_values;
        _time_from_start = time_series->_time_from_start;
        _value_dim       = time_series->_value_dim;
        _value_labels    = time_series->_value_labels;
        return true;
    }

    if (time_series->_time_from_start < getTimeFromStart())
    {
        PRINT_ERROR("inserting only allows newer time series");
        return false;
    }

    // calculate time gap between the end of this and the start of the new time series
    double dT     = time_series->_time_from_start - getFinalTimeFromStart();
    int start_idx = 0;

    if (dT > 0.0)
    {
        // dt of last state
        double dt = _time[_time.size() - 1] - _time[_time.size() - 2];

        // fill the gap with extrapolation
        while (dT - dt > 1e-5)
        {
            add(getFinalTime() + dt, getValues(getTimeDimension() - 1));
            dT -= dt;
        }

        // reserve extra space
        _time.reserve(_time.size() + time_series->_time.size());
        _values.reserve(_values.size() + time_series->_values.size());

        // append new time and values
        double t0 = getFinalTime() + dT;
        for (int i = start_idx; i < time_series->_time.size(); ++i)
        {
            add(t0 + time_series->_time[i], time_series->getValues(i));
        }
    }
    else
    {
        if (overwrite)
        {
            // find idx to start overwriting
            double start_time    = time_series->_time_from_start;
            double tfs           = _time_from_start;
            auto start           = std::find_if(_time.begin(), _time.end(), [start_time, tfs](double val) { return tfs + val >= start_time; });
            int values_start_idx = _value_dim * std::distance(_time.begin(), start);

            // remove old data
            _time.erase(start, _time.end());
            _values.erase(_values.begin() + values_start_idx, _values.end());
        }
        else
        {
            // find idx to start copying
            double start_time = getFinalTimeFromStart();
            double tfs        = time_series->_time_from_start;
            auto start =
                std::find_if(time_series->_time.begin(), time_series->_time.end(), [start_time, tfs](double val) { return tfs + val > start_time; });
            start_idx = std::distance(time_series->_time.begin(), start);
        }

        // reserve extra space
        _time.reserve(_time.size() + time_series->_time.size());
        _values.reserve(_values.size() + time_series->_values.size());

        // append new time and values
        double t0 = time_series->_time_from_start - getTimeFromStart();
        for (int i = start_idx; i < time_series->_time.size(); ++i)
        {
            add(t0 + time_series->_time[i], time_series->getValues(i));
        }
    }

    return true;
}

void TimeSeries::prune(double time)
{
    if (_time.empty() || time < _time[0]) return;
    if (time > getFinalTime())
    {
        clear();
        return;
    }

    // find next index up to which the data should be removed
    auto it = std::find_if(_time.begin(), _time.end(), [time](double val) { return val > time; });
    int idx = (std::distance(_time.begin(), it)) * _value_dim;

    // store current value to add it later
    Eigen::VectorXd x(_value_dim);
    getValuesInterpolate(time, x);

    // remove data
    _time.erase(_time.begin(), it);
    _values.erase(_values.begin(), _values.begin() + idx);

    // add current value to the back and rotate to the front
    add(time, x);
    std::rotate(_time.rbegin(), _time.rbegin() + 1, _time.rend());
    std::rotate(_values.rbegin(), _values.rbegin() + _value_dim, _values.rend());

    // shift time values so time series starts again at 0
    std::for_each(_time.begin(), _time.end(), [time](double& val) { val -= time; });
    _time_from_start += time;
}

void TimeSeries::clear()
{
    _time.clear();
    _values.clear();
    _value_dim       = 0;
    _time_from_start = 0.0;
}

std::vector<double> TimeSeries::getValues(int time_idx) const
{
    assert(time_idx < _time.size());
    std::vector<double> temp;
    const int value_idx_begin = _value_dim * time_idx;
    const int value_idx_end   = value_idx_begin + _value_dim;
    temp.assign(_values.begin() + value_idx_begin, _values.begin() + value_idx_end);
    return temp;
}

Eigen::Map<const Eigen::VectorXd> TimeSeries::getValuesMap(int time_idx) const
{
    assert(time_idx < _time.size());
    const int value_idx_begin = _value_dim * time_idx;
    return Eigen::Map<const Eigen::VectorXd>(_values.data() + value_idx_begin, _value_dim);
}

bool TimeSeries::getValuesInterpolate(double time, Eigen::Ref<Eigen::VectorXd> values, Interpolation interpolation, Extrapolation extrapolation,
                                      double tolerance) const
{
    if (_time.empty()) return false;
    if (time < 0)
    {
        PRINT_ERROR("Accessing the timeseries before it starts");
        return false;
    }

    auto it = std::find_if(_time.begin(), _time.end(), [time](double val) { return val >= time; });  // find next time interval
    if (it == _time.end())
    {
        switch (extrapolation)
        {
            case Extrapolation::NoExtrapolation: {
                values.setZero();
                break;
            }
            case Extrapolation::ZeroOrderHold: {
                values = getValuesMap(getTimeDimension() - 1);
                return true;
            }
            default: {
                PRINT_ERROR("TimeSeries::valuesInterpolate(): desired extrapolation method not implemented.");
                return false;
            }
        }
    }
    // interpolate
    int idx = (int)std::distance(_time.begin(), it);
    if (idx >= getTimeDimension()) idx = getTimeDimension() - 1;  // this might occure with floating point comparison
    if (std::abs(time - _time[idx]) < tolerance)
    {
        // we are close to the next value, in particular idx
        values = getValuesMap(idx);
        return true;
    }
    // assert idx > 0

    // okay, so now we have the above case val > time -> so idx corresponds to the next sample.
    double dt = time - _time[idx - 1];
    PRINT_ERROR_COND_NAMED(dt < 0, "dt < 0 in interpolation. This cannot be correct.");

    switch (interpolation)
    {
        case Interpolation::ZeroOrderHold: {
            values = getValuesMap(idx - 1);  // since dt > 0 -> we use the previous value
            break;
        }
        case Interpolation::Linear: {
            double dt_data   = _time[idx] - _time[idx - 1];
            double dt_frac   = dt / dt_data;
            values.noalias() = getValuesMap(idx - 1) + dt_frac * (getValuesMap(idx) - getValuesMap(idx - 1));

            break;
        }
        default: {
            PRINT_ERROR("TimeSeries::valuesInterpolate(): desired interpolation method not implemented.");
            return false;
        }
    }
    return true;
}

double TimeSeries::computeMeanOverall() { return getValuesMatrixView().mean(); }

void TimeSeries::computeMeanCwise(Eigen::Ref<Eigen::VectorXd> mean_values)
{
    if (mean_values.size() != getValueDimension())
    {
        PRINT_ERROR("TimeSeries::computeMeanCwise(): provided mean_values vector does not match value dimension");
        return;
    }
    mean_values = getValuesMatrixView().rowwise().mean();
}

void TimeSeries::normalize(Normalization norm_method, bool value_cwise)
{
    if (isEmpty()) return;

    if (value_cwise)
    {
        for (int i = 0; i < getValueDimension(); ++i) normalize(norm_method, i);
        return;
    }

    switch (norm_method)
    {
        case Normalization::FirstValue: {
            double first_value = _values.front();
            if (first_value != 0)
                getValuesMatrixView() /= first_value;
            else
                getValuesMatrixView().fill(MHP_PLANNER_INF_DBL);
            break;
        }
        case Normalization::AbsoluteMaximum: {
            double max_value = getValuesMatrixView().lpNorm<Eigen::Infinity>();
            if (max_value != 0)
                getValuesMatrixView() /= max_value;
            else
                getValuesMatrixView().fill(MHP_PLANNER_INF_DBL);
            break;
        }
        case Normalization::Maximum: {
            double max_value = getValuesMatrixView().maxCoeff();
            if (max_value != 0)
                getValuesMatrixView() /= max_value;
            else
                getValuesMatrixView().fill(MHP_PLANNER_INF_DBL);
            break;
        }
        case Normalization::Mean: {
            double mean_value = computeMeanOverall();
            if (mean_value != 0)
                getValuesMatrixView() /= mean_value;
            else
                getValuesMatrixView().fill(MHP_PLANNER_INF_DBL);
            break;
        }
        default: {
            PRINT_ERROR("TimeSeries::normalize(): selected method not implemented.");
            break;
        }
    }
}

void TimeSeries::normalize(Normalization norm_method, int value_idx)
{
    if (isEmpty()) return;
    if (value_idx >= getValueDimension())
    {
        PRINT_ERROR("TimeSeries::normalize(): specified value_idx does not match getValueDimension().");
        return;
    }

    switch (norm_method)
    {
        case Normalization::FirstValue: {
            double first_value = getValues(0)[value_idx];
            if (first_value != 0)
                getValuesMatrixView().row(value_idx) /= first_value;
            else
                getValuesMatrixView().row(value_idx).fill(MHP_PLANNER_INF_DBL);
            break;
        }
        case Normalization::AbsoluteMaximum: {
            double max_value = getValuesMatrixView().row(value_idx).lpNorm<Eigen::Infinity>();
            if (max_value != 0)
                getValuesMatrixView().row(value_idx) /= max_value;
            else
                getValuesMatrixView().row(value_idx).fill(MHP_PLANNER_INF_DBL);
            break;
        }
        case Normalization::Maximum: {
            double max_value = getValuesMatrixView().row(value_idx).maxCoeff();
            if (max_value != 0)
                getValuesMatrixView().row(value_idx) /= max_value;
            else
                getValuesMatrixView().row(value_idx).fill(MHP_PLANNER_INF_DBL);
            break;
        }
        case Normalization::Mean: {
            double mean_value = getValuesMatrixView().row(value_idx).mean();
            if (mean_value != 0)
                getValuesMatrixView().row(value_idx) /= mean_value;
            else
                getValuesMatrixView().row(value_idx).fill(MHP_PLANNER_INF_DBL);
            break;
        }
        default: {
            PRINT_ERROR("TimeSeries::normalize(): selected method not implemented.");
            break;
        }
    }
}

void TimeSeries::print(int precision) const { PRINT_INFO(std::endl << std::fixed << std::setprecision(precision) << *this); }

std::ostream& operator<<(std::ostream& out, const TimeSeries& ts)
{
    if (ts.isEmpty())
    {
        out << "TimeSeries is empty." << std::endl;
        return out;
    }
    for (int i = 0; i < ts.getTimeDimension(); ++i)
    {
        // TODO(roesmann) include std::precision here?
        out << "time: " << ts.getTime()[i] << "\t values: " << ts.getValuesMap(i).transpose() << std::endl;
    }
    return out;
}

void TimeSeriesSequence::setValueDimension(int value_dim)
{
    if (value_dim != _value_dim)
    {
        clear();
        _value_dim = value_dim;
    }
}

bool TimeSeriesSequence::add(TimeSeries::Ptr ts)
{
    if (!ts) return false;

    if (_ts_sequence.empty())
    {
        setValueDimension(ts->getValueDimension());
    }
    else if (ts->getValueDimension() != _value_dim)
    {
        PRINT_ERROR("TimeSeriesSequence::add(): cannot add TimeSeries since its dimension differs with previously added ones.");
        return false;
    }
    _ts_sequence.push_back(ts);
    return true;
}

void TimeSeriesSequence::clear()
{
    _ts_sequence.clear();
    _value_dim = 0;
}

void TimeSeriesSequence::sortByTimeFromStart()
{
    std::sort(_ts_sequence.begin(), _ts_sequence.end(),
              [](const TimeSeries::Ptr& ts1, const TimeSeries::Ptr& ts2) { return (ts1->getTimeFromStart() < ts2->getTimeFromStart()); });
}

}  // namespace mhp_planner
