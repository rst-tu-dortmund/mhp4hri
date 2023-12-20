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

#ifndef SRC_SYSTEMS_INCLUDE_MHP_PLANNER_SYSTEMS_STANDARD_FILTERS_H_
#define SRC_SYSTEMS_INCLUDE_MHP_PLANNER_SYSTEMS_STANDARD_FILTERS_H_

#include <mhp_planner/systems/filter_interface.h>

#include <deque>
#include <memory>

namespace mhp_planner {

/**
 * @brief Moving Average Filter
 *
 * @ingroup systems
 *
 * This class implements a moving average filter based on a specified window size.
 *
 * @remark This interface is provided with factory support (FilterFactory).
 *
 * @see FilterInterface MovingMedianFilter MovingLeastSquaresFilter
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class MovingAverageFilter : public FilterInterface
{
 public:
    using Ptr = std::shared_ptr<MovingAverageFilter>;

    // implements interface method
    FilterInterface::Ptr getInstance() const override { return std::make_shared<MovingAverageFilter>(); }

    // implements interface method
    double filter(double t, double value) override;

    // implements interface method
    void reset() override;

    void setWindowSize(int window_size) { _window_size = window_size; }

    bool fromParameterServer(const std::string& ns) override;

 private:
    int _window_size = 5;

    std::deque<double> _values;
    bool _first_value = true;
};

FACTORY_REGISTER_FILTER(MovingAverageFilter)

/**
 * @brief Moving Median Filter
 *
 * @ingroup systems
 *
 * This class implements a moving median filter based on a specified window size.
 *
 * @remark This interface is provided with factory support (FilterFactory).
 *
 * @see FilterInterface MovingAverageFilter MovingLeastSquaresFilter
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class MovingMedianFilter : public FilterInterface
{
 public:
    using Ptr = std::shared_ptr<MovingMedianFilter>;

    // implements interface method
    FilterInterface::Ptr getInstance() const override { return std::make_shared<MovingMedianFilter>(); }

    // implements interface method
    double filter(double t, double value) override;

    // implements interface method
    void reset() override;

    void setWindowSize(int window_size) { _window_size = window_size; }

    bool fromParameterServer(const std::string& ns) override;

 private:
    int _window_size = 5;

    std::deque<double> _values;
    bool _first_value = true;
};

FACTORY_REGISTER_FILTER(MovingMedianFilter)

/**
 * @brief Moving LeastSquares Filter
 *
 * @ingroup systems
 *
 * This class implements a moving filter which solves an unconstrained
 * least squares problem along the specified window size.
 * It returns the last point of the regression result.
 *
 * @remark This interface is provided with factory support (FilterFactory).
 *
 * @see FilterInterface MovingAverageFilter MovingMedianFilter
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class MovingLeastSquaresFilter : public FilterInterface
{
 public:
    using Ptr = std::shared_ptr<MovingMedianFilter>;

    // implements interface method
    FilterInterface::Ptr getInstance() const override { return std::make_shared<MovingLeastSquaresFilter>(); }

    // implements interface method
    double filter(double t, double value) override;

    // implements interface method
    void reset() override;

    void setWindowSize(int window_size) { _window_size = window_size; }
    void setPolynomialOrder(int order) { _order = order; }

    bool fromParameterServer(const std::string& ns) override;

 protected:
    void allocateMemory();
    void buildRegressionMatrix();

 private:
    // parameters
    int _window_size = 5;
    int _order       = 3;

    // internal states
    Eigen::VectorXd _values;
    Eigen::VectorXd _time;
    Eigen::MatrixXd _regression_matrix;
    int _num_values = 0;
};

FACTORY_REGISTER_FILTER(MovingLeastSquaresFilter)

}  // namespace mhp_planner

#endif  // SRC_SYSTEMS_INCLUDE_MHP_PLANNER_SYSTEMS_STANDARD_FILTERS_H_
