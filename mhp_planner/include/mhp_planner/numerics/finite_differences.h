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

#ifndef SRC_NUMERICS_INCLUDE_MHP_PLANNER_NUMERICS_FINITE_DIFFERENCES_H_
#define SRC_NUMERICS_INCLUDE_MHP_PLANNER_NUMERICS_FINITE_DIFFERENCES_H_

#include <mhp_planner/numerics/finite_differences_interface.h>

#include <functional>
#include <memory>

namespace mhp_planner {

/**
 * @brief Finite differences via forward differences
 *
 * @ingroup numerics
 *
 * Forward differences approximate \f$ \dot{x} = f(x) \f$ in the following manner:
 * \f[
 *    \frac{x_{k+1} - x_k}{\delta} = f(x_k)
 * \f]
 *
 * @see FiniteDifferencesInterface CentralDifferences NumericalIntegratorExplicitInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class ForwardDifferences : public FiniteDifferencesInterface
{
 public:
    // Implements interface method
    FiniteDifferencesInterface::Ptr getInstance() const override { return std::make_shared<ForwardDifferences>(); }

    template <typename IncFun, typename EvalFun>
    static void jacobian(IncFun inc_fun, EvalFun eval_fun, Eigen::Ref<Eigen::MatrixXd> jacobian);

    template <typename IncFun, typename EvalFun>
    static void hessian(IncFun inc_fun, EvalFun eval_fun, int dim_f, Eigen::Ref<Eigen::MatrixXd> hessian, const double* multipliers = nullptr);

    template <typename IncFun, typename EvalFun>
    void jacobianHessian(IncFun inc_fun, EvalFun eval_fun, Eigen::Ref<Eigen::MatrixXd> jacobian, Eigen::Ref<Eigen::MatrixXd> hessian,
                         const double* multipliers = nullptr);

    // Implements interface method
    void computeJacobian(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::Ref<Eigen::VectorXd>)> eval_fun,
                         Eigen::Ref<Eigen::MatrixXd> jacobian) override;

    // Implements interface method
    void computeJacobian2(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::VectorXd&)> eval_fun,
                          Eigen::Ref<Eigen::MatrixXd> jacobian) override;

    // Implements interface method
    void computeHessian(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::Ref<Eigen::VectorXd>)> eval_fun, int dim_f,
                        Eigen::Ref<Eigen::MatrixXd> hessian, const double* multipliers = nullptr) override;

    // Implements interface method
    void computeHessian2(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::VectorXd&)> eval_fun, int dim_f,
                         Eigen::Ref<Eigen::MatrixXd> hessian, const double* multipliers = nullptr) override;

    // Implements interface method
    void computeJacobianAndHessian(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::Ref<Eigen::VectorXd>)> eval_fun,
                                   Eigen::Ref<Eigen::MatrixXd> jacobian, Eigen::Ref<Eigen::MatrixXd> hessian,
                                   const double* multipliers = nullptr) override;

    // Implements interface method
    void computeJacobianAndHessian2(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::VectorXd&)> eval_fun,
                                    Eigen::Ref<Eigen::MatrixXd> jacobian, Eigen::Ref<Eigen::MatrixXd> hessian,
                                    const double* multipliers = nullptr) override;
};
FACTORY_REGISTER_FINITE_DIFFERENCES(ForwardDifferences)

/**
 * @brief Finite differences via central differences
 *
 * @ingroup numerics
 *
 * Central differences approximate \f$ \dot{x} = f(x) \f$ in the following manner:
 * \f[
 *    \frac{x_{k+1} - x_{k-1}{2 \delta} = f(x_k)
 * \f]
 *
 * @see FiniteDifferencesInterface CentralDifferences NumericalIntegratorExplicitInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class CentralDifferences : public FiniteDifferencesInterface
{
 public:
    // Implements interface method
    FiniteDifferencesInterface::Ptr getInstance() const override { return std::make_shared<CentralDifferences>(); }

    template <typename IncFun, typename EvalFun>
    static void jacobian(IncFun inc_fun, EvalFun eval_fun, Eigen::Ref<Eigen::MatrixXd> jacobian);

    template <typename IncFun, typename EvalFun>
    static void hessian(IncFun inc_fun, EvalFun eval_fun, int dim_f, Eigen::Ref<Eigen::MatrixXd> hessian, const double* multipliers = nullptr);
    // template <typename IncFun, typename EvalFun, typename WeightFun = double(int)>
    // static void hessian(IncFun inc_fun, EvalFun eval_fun, Eigen::Ref<Eigen::MatrixXd> hessian, int dim_f = 1, WeightFun weight_fun = 0);

    template <typename IncFun, typename EvalFun>
    void jacobianHessian(IncFun inc_fun, EvalFun eval_fun, Eigen::Ref<Eigen::MatrixXd> jacobian, Eigen::Ref<Eigen::MatrixXd> hessian,
                         const double* multipliers = nullptr);

    // Implements interface method
    void computeJacobian(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::Ref<Eigen::VectorXd>)> eval_fun,
                         Eigen::Ref<Eigen::MatrixXd> jacobian) override;

    // Implements interface method
    void computeJacobian2(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::VectorXd&)> eval_fun,
                          Eigen::Ref<Eigen::MatrixXd> jacobian) override;

    // Implements interface method
    void computeHessian(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::Ref<Eigen::VectorXd>)> eval_fun, int dim_f,
                        Eigen::Ref<Eigen::MatrixXd> hessian, const double* multipliers = nullptr) override;

    // Implements interface method
    void computeHessian2(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::VectorXd&)> eval_fun, int dim_f,
                         Eigen::Ref<Eigen::MatrixXd> hessian, const double* multipliers = nullptr) override;

    // Implements interface method
    void computeJacobianAndHessian(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::Ref<Eigen::VectorXd>)> eval_fun,
                                   Eigen::Ref<Eigen::MatrixXd> jacobian, Eigen::Ref<Eigen::MatrixXd> hessian,
                                   const double* multipliers = nullptr) override;

    // Implements interface method
    void computeJacobianAndHessian2(std::function<void(int, const double&)> inc_fun, std::function<void(Eigen::VectorXd&)> eval_fun,
                                    Eigen::Ref<Eigen::MatrixXd> jacobian, Eigen::Ref<Eigen::MatrixXd> hessian,
                                    const double* multipliers = nullptr) override;
};
FACTORY_REGISTER_FINITE_DIFFERENCES(CentralDifferences)

}  // namespace mhp_planner

#include <mhp_planner/numerics/finite_differences.hpp>  // NOLINT(build/include_order)

#endif  // SRC_NUMERICS_INCLUDE_MHP_PLANNER_NUMERICS_FINITE_DIFFERENCES_H_
