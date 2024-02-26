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

#ifndef SRC_NUMERICS_INCLUDE_MHP_PLANNER_NUMERICS_FINITE_DIFFERENCES_INTERFACE_H_
#define SRC_NUMERICS_INCLUDE_MHP_PLANNER_NUMERICS_FINITE_DIFFERENCES_INTERFACE_H_

#include <mhp_planner/core/factory.h>
#include <mhp_planner/core/time.h>
#include <mhp_planner/core/types.h>

#include <functional>
#include <memory>

namespace mhp_planner {

/**
 * @brief Interface class for finite difference approaches
 *
 * @ingroup numerics
 *
 * This base class provides an interface for computing
 * finite differences as derivative approximation
 * for some relevant functions.
 * For instance for computing the Jacobian matrices
 * of the system dynamics function $\f f(x, u) \$f
 * w.r.t. \f$ x \f$ and/or \f$ u \f$.
 *
 * @remark This interface is provided with factory support (FiniteDifferencesFactory).
 *
 * @see ForwardDifferences CentralDifferences NumericalIntegratorExplicitInterface
 *      NumericalIntegratorExplicitInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class FiniteDifferencesInterface
{
 public:
    using Ptr  = std::shared_ptr<FiniteDifferencesInterface>;
    using UPtr = std::unique_ptr<FiniteDifferencesInterface>;

    using StateVector = Eigen::VectorXd;
    using InputVector = Eigen::VectorXd;

    //! Virtual destructor
    virtual ~FiniteDifferencesInterface() {}
    //! Return a newly allocated instances of the inherited class.
    virtual FiniteDifferencesInterface::Ptr getInstance() const = 0;

    virtual void computeJacobian(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::Ref<Eigen::VectorXd>)> eval_fun,
                                 Eigen::Ref<Eigen::MatrixXd> jacobian) = 0;

    virtual void computeJacobian2(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::VectorXd&)> eval_fun,
                                  Eigen::Ref<Eigen::MatrixXd> jacobian) = 0;

  
    virtual void computeHessian(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::Ref<Eigen::VectorXd>)> eval_fun, int dim_f,
                                Eigen::Ref<Eigen::MatrixXd> hessian, const double* multipliers = nullptr) = 0;

    virtual void computeHessian2(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::VectorXd&)> eval_fun, int dim_f,
                                 Eigen::Ref<Eigen::MatrixXd> hessian, const double* multipliers = nullptr) = 0;


    virtual void computeJacobianAndHessian(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::Ref<Eigen::VectorXd>)> eval_fun,
                                           Eigen::Ref<Eigen::MatrixXd> jacobian, Eigen::Ref<Eigen::MatrixXd> hessian,
                                           const double* multipliers = nullptr) = 0;


    virtual void computeJacobianAndHessian2(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::VectorXd&)> eval_fun,
                                            Eigen::Ref<Eigen::MatrixXd> jacobian, Eigen::Ref<Eigen::MatrixXd> hessian,
                                            const double* multipliers = nullptr) = 0;
};

using FiniteDifferencesFactory = Factory<FiniteDifferencesInterface>;
#define FACTORY_REGISTER_FINITE_DIFFERENCES(type) FACTORY_REGISTER_OBJECT(type, FiniteDifferencesInterface)

}  // namespace mhp_planner

#endif  // SRC_NUMERICS_INCLUDE_MHP_PLANNER_NUMERICS_FINITE_DIFFERENCES_INTERFACE_H_
