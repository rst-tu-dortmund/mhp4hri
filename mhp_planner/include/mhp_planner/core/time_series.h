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

#ifndef SRC_CORE_INCLUDE_MHP_PLANNER_CORE_TIME_SERIES_H_
#define SRC_CORE_INCLUDE_MHP_PLANNER_CORE_TIME_SERIES_H_

#include <mhp_planner/core/types.h>

#include <memory>
#include <string>
#include <vector>

namespace mhp_planner {

/**
 * @brief Time Series (trajectory resp. sequence of values w.r.t. time)
 *
 * @ingroup core
 *
 * A time series is meant to represent a sequence of vectors of floating point numbers
 * with individual time stamps. A time series might also be interpreted as a
 * discrete-time trajectory.
 *
 * @see TimeSeriesSequence TimeSeriesSignal TimeSeriesSequenceSignal
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class TimeSeries
{
 public:
    //! List of available interpolation methods
    enum class Interpolation { ZeroOrderHold, Linear };
    //! List of available interpolation methods
    enum class Extrapolation { NoExtrapolation, ZeroOrderHold };
    //! List of available normalizations
    enum class Normalization { FirstValue, AbsoluteMaximum, Maximum, Mean };

    using Ptr               = std::shared_ptr<TimeSeries>;
    using ConstPtr          = std::shared_ptr<const TimeSeries>;
    using ValuesMatMap      = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>>;
    using ValuesMatConstMap = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>>;
    using TimeVecMap        = Eigen::Map<Eigen::VectorXd>;
    using TimeVecConstMap   = Eigen::Map<const Eigen::VectorXd>;

    //! Default constructor
    TimeSeries() = default;
    //! Construct empty time series with a dresired value vector dimension
    explicit TimeSeries(int value_dim) : _value_dim(value_dim) {}

    virtual ~TimeSeries() {}

    //! Return dimension of the value vector
    int getValueDimension() const { return _value_dim; }
    //! Return dimension of the time vector
    int getTimeDimension() const { return (int)_time.size(); }
    //! Determine if time series is empty
    bool isEmpty() const { return _time.empty(); }
    //! Return if the underlying raw format is in Column-Major layout
    bool isColumnMajor() const { return true; }

    // manipulation
    //! Change value dimension (warning: clears existing values)
    void setValueDimension(int value_dim);
    //! Allocate memory for a given time and value vector dimension
    void reserve(int time_dim, int value_dim);

    //! Add time and value vector pair (STL version)
    bool add(double time, const std::vector<double>& values);
    //! Add time and value vector pair (Eigen version)
    bool add(double time, const Eigen::Ref<const Eigen::VectorXd>& values);

    bool set(const Eigen::Ref<const Eigen::VectorXd>& time, const Eigen::Ref<const Eigen::MatrixXd>& values_matrix, double time_from_start = 0.0);

    bool set(const std::vector<double>& time, const std::vector<Eigen::VectorXd>& values_vector, double time_from_start = 0.0);

    bool insert(const TimeSeries::ConstPtr time_series, bool overwrite = true);

    //! Clear all time and values with t + time_from_start <= time
    void prune(double time);

    //! Clear time series (except valueLabels())
    void clear();

    //! Read individual value labels (non-empty if provided)
    const std::vector<std::string>& getValueLabels() const { return _value_labels; }
    //! Access (modify) value labels for each component
    std::vector<std::string>& getValueLabelsRef() { return _value_labels; }

    // accessors
    //! Obtain value vector (copy) at a given time idx (< getTimeDimension())
    std::vector<double> getValues(int time_idx) const;
    //! Read access to a value vector at a given time idx (< getTimeDimension) without copying
    Eigen::Map<const Eigen::VectorXd> getValuesMap(int time_idx) const;

    virtual bool getValuesInterpolate(double time, Eigen::Ref<Eigen::VectorXd> values, Interpolation interpolation = Interpolation::Linear,
                                      Extrapolation extrapolate = Extrapolation::NoExtrapolation, double tolerance = 1e-6) const;

    //! Read access to the complete values matrix in column-major format [getValueDimension() x getTimeDimension()]
    const std::vector<double>& getValues() const { return _values; }
    //! Access to the complete values matrix in column-major format [getValueDimension() x getTimeDimension()] (warning, modify only with 100% care)
    std::vector<double>& getValuesRef() { return _values; }
    //! Read access to the complete values matrix in Eigen matrix format [getValueDimension() x getTimeDimension()]
    ValuesMatConstMap getValuesMatrixView() const { return ValuesMatConstMap(_values.data(), _value_dim, getTimeDimension()); }
    //! Access to the complete values matrix in Eigen matrix format [getValueDimension() x getTimeDimension()] (warning, modify only with 100% care)
    ValuesMatMap getValuesMatrixView() { return ValuesMatMap(_values.data(), _value_dim, getTimeDimension()); }

    //! Read access to the underlying time values [getTimeDimension() x 1]
    const std::vector<double>& getTime() const { return _time; }
    //! Write access to the underlying time values [getTimeDimension() x 1] (warning, modify only with 100% care)
    std::vector<double>& getTimeRef() { return _time; }
    //! Read access to the underlying time values in Eigen vector format [getTimeDimension() x 1]
    TimeVecConstMap getTimeVectorView() const { return TimeVecConstMap(_time.data(), _time.size()); }
    //! Write access to the underlying time values in Eigen vector format [getTimeDimension() x 1] (warning, modify only with 100% care)
    TimeVecMap getTimeVectorView() { return TimeVecMap(_time.data(), _time.size()); }

    //! Get time from start (offset to all time stamps in time())
    const double& getTimeFromStart() const { return _time_from_start; }
    //! Set time from start (offset to all time stamps in time())
    void setTimeFromStart(double tfs) { _time_from_start = tfs; }

    //! Get total time duration of the trajectory (assumes that all values are sorted with ascending time)
    double getFinalTime() const { return _time.empty() ? 0.0 : _time.back(); }

    //! Get total time duration of the trajectory from start (assumes that all values are sorted with ascending time)
    double getFinalTimeFromStart() const { return getFinalTime() + _time_from_start; }

    //! Compute and return the mean value of all values among all dimensions
    virtual double computeMeanOverall();
   
    virtual void computeMeanCwise(Eigen::Ref<Eigen::VectorXd> mean_values);

    virtual void normalize(Normalization norm_method, bool value_cwise = true);

    virtual void normalize(Normalization norm_method, int value_idx);

    //! Print the current time series content to console (with a desired precision for decimal numbers)
    void print(int precision = 2) const;

    friend std::ostream& operator<<(std::ostream& out, const TimeSeries& ts);

    // overloaded operators
    // friend std::ostream& operator<<(std::ostream& os, const std::vector<double>& values) {add()}

 protected:
    // We implement our own column-major matrix layout with std::vector<double>
    // instead of relying on Eigens implementation Eigen::MatrixXd
    // since we want to support a dedicated container capacity which might
    // be larger than the current size (see std::vector::reserve).
    // We assume that data is frequently appended/added during data recording.
    // Indeed, Eigen provides a conservativeResize method which utilizes std::reallocate
    // whenever columns are added in a column-major layout, but it is not guaranteed that
    // there is enough memory capacity to avoid copies completely.
    int _value_dim = 0;           // we could infer this from values.size()/time.size() but we do not want to compute this every lookup
    std::vector<double> _values;  //!< value x time [column major], column: set of values corresponding to the given time idx
    std::vector<double> _time;

    double _time_from_start = 0.0;  //!< Speciy an offset for the time sequence _time;

    std::vector<std::string> _value_labels;  //!< label values should be empty or valueDimension()
};

/**
 * @brief Sequence of time series objects
 *
 * @ingroup core
 *
 * Individual time series might be sorted w.r.t different
 * TimeSeries::timeFromStart() values e.g. in order to
 * indicate planned or predicted trajectories at each individual
 * sampling interval.
 *
 * @see TimeSeries TimeSeriesSequenceSignal TimeSeriesSignal
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class TimeSeriesSequence
{
 public:
    using Ptr      = std::shared_ptr<TimeSeriesSequence>;
    using ConstPtr = std::shared_ptr<const TimeSeriesSequence>;

    //! Default constructor
    TimeSeriesSequence() = default;

    //! Construct empty time series sequence with a desired value vector dimension
    explicit TimeSeriesSequence(int value_dim) : _value_dim(value_dim) {}

    //! Return dimension of the value vector
    int getValueDimension() const { return _value_dim; }
    //! Determine if no time series is available
    bool isEmpty() const { return _ts_sequence.empty(); }

    // manipulation
    //! Change value dimension (warning: clears existing values)
    void setValueDimension(int value_dim);

    //! Add shared instance of a time series (without copying)
    bool add(TimeSeries::Ptr ts);

    //! Read access to the underlying sequence of time series (shared instances)
    const std::vector<TimeSeries::Ptr>& getSequence() const { return _ts_sequence; }

    //! Erase all time series
    void clear();

    //! Ascending sort of the time series objects according to their TimeSeries::timeFromStart()
    void sortByTimeFromStart();

 private:
    std::vector<TimeSeries::Ptr> _ts_sequence;

    int _value_dim = 0;  //!< We enforce a unique value dimension for all internal time_series objects
};

}  // namespace mhp_planner

#endif  // SRC_CORE_INCLUDE_MHP_PLANNER_CORE_TIME_SERIES_H_
