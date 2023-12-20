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

#ifndef SRC_NUMERICS_INCLUDE_MHP_PLANNER_NUMERICS_FINITE_DIFFERENCES_COLLOCATION_H_
#define SRC_NUMERICS_INCLUDE_MHP_PLANNER_NUMERICS_FINITE_DIFFERENCES_COLLOCATION_H_

#include <mhp_planner/core/factory.h>
#include <mhp_planner/core/time.h>
#include <mhp_planner/core/types.h>

#include <functional>
#include <memory>

namespace mhp_planner {

/**
 * @brief Interface class for finite difference based collocation
 *
 * @ingroup numerics
 *
 * This base class provides an interface for approximating
 * continuous-time dynamics between two consecutive points.
 *
 * Online references:
 * http://www.control.lth.se/media/Education/DoctorateProgram/2011/OptimizationWithCasadi/lecture4b_short_slides.pdf
 * https://mec560sbu.github.io/2016/09/30/direct_collocation/
 * http://epubs.siam.org/doi/pdf/10.1137/16M1062569
 *
 * @see ForwardDiffCollocation BackwardDiffCollocation MidpointDiffCollocation
 *      CrankNicolsonDiffCollocation CubicSplineCollocation DynamicsEvalInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class FiniteDifferencesCollocationInterface
{
 public:
    using Ptr  = std::shared_ptr<FiniteDifferencesCollocationInterface>;
    using UPtr = std::unique_ptr<FiniteDifferencesCollocationInterface>;

    using StateVector = Eigen::VectorXd;
    using InputVector = Eigen::VectorXd;

    //! Virtual destructor
    virtual ~FiniteDifferencesCollocationInterface() = default;
    //! Return a newly allocated instances of the inherited class.
    virtual FiniteDifferencesCollocationInterface::Ptr getInstance() const = 0;

    //! Get access to the associated factory
    static Factory<FiniteDifferencesCollocationInterface>& getFactory() { return Factory<FiniteDifferencesCollocationInterface>::instance(); }

    virtual void computeEqualityConstraint(const StateVector& x1, const InputVector& u1, const StateVector& x2, double dt,
                                           const SystemDynamicsInterface& system, Eigen::Ref<Eigen::VectorXd> error) = 0;

    virtual void fromParameterServer(const std::string& ns) = 0;
};
using FiniteDifferencesCollocationFactory = Factory<FiniteDifferencesCollocationInterface>;
#define FACTORY_REGISTER_FD_COLLOCATION(type) FACTORY_REGISTER_OBJECT(type, FiniteDifferencesCollocationInterface)

/**
 * @brief Collocation via forward differences
 *
 * @ingroup numerics
 *
 * Forward differences approximate \f$ \dot{x} = f(x, u) \f$ in the following manner:
 * \f[
 *    \frac{x_{k+1} - x_k}{\delta T} = f(x_k, u_k)
 * \f]
 *
 * @see FiniteDifferencesCollocationInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class ForwardDiffCollocation : public FiniteDifferencesCollocationInterface
{
 public:
    // Implements interface method
    FiniteDifferencesCollocationInterface::Ptr getInstance() const override { return std::make_shared<ForwardDiffCollocation>(); }

    // Implements interface method
    void computeEqualityConstraint(const StateVector& x1, const InputVector& u1, const StateVector& x2, double dt,
                                   const SystemDynamicsInterface& system, Eigen::Ref<Eigen::VectorXd> error) override
    {
        assert(error.size() == x1.size());
        assert(dt > 0 && "dt must be greater then zero!");

        system.dynamics(x1, u1, error);
        error -= (x2 - x1) / dt;
    }

    void fromParameterServer(const std::string& ns) override{};
};
FACTORY_REGISTER_FD_COLLOCATION(ForwardDiffCollocation)

/**
 * @brief Collocation via backward differences
 *
 * @ingroup numerics
 *
 * Backward differences approximate \f$ \dot{x} = f(x, u) \f$ in the following manner:
 * \f[
 *    \frac{x_{k+1} - x_k}{\delta T} = f(x_{k+1}, u_k)
 * \f]
 *
 * @see FiniteDifferencesCollocationInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class BackwardDiffCollocation : public FiniteDifferencesCollocationInterface
{
 public:
    // Implements interface method
    FiniteDifferencesCollocationInterface::Ptr getInstance() const override { return std::make_shared<BackwardDiffCollocation>(); }

    // Implements interface method
    void computeEqualityConstraint(const StateVector& x1, const InputVector& u1, const StateVector& x2, double dt,
                                   const SystemDynamicsInterface& system, Eigen::Ref<Eigen::VectorXd> error) override
    {
        assert(error.size() == x1.size());
        assert(dt > 0 && "dt must be greater then zero!");

        system.dynamics(x2, u1, error);
        error -= (x2 - x1) / dt;
    }

    void fromParameterServer(const std::string& ns) override{};
};
FACTORY_REGISTER_FD_COLLOCATION(BackwardDiffCollocation)

/**
 * @brief Collocation via midpoint differences
 *
 * @ingroup numerics
 *
 * Midpoint differences approximate \f$ \dot{x} = f(x, u) \f$ in the following manner:
 * \f[
 *    \frac{x_{k+1} - x_k}{\delta T} = f(0.5*(x_k + x_{k+1}), u_k)
 * \f]
 *
 * @see FiniteDifferencesCollocationInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class MidpointDiffCollocation : public FiniteDifferencesCollocationInterface
{
 public:
    // Implements interface method
    FiniteDifferencesCollocationInterface::Ptr getInstance() const override { return std::make_shared<MidpointDiffCollocation>(); }

    // Implements interface method
    void computeEqualityConstraint(const StateVector& x1, const InputVector& u1, const StateVector& x2, double dt,
                                   const SystemDynamicsInterface& system, Eigen::Ref<Eigen::VectorXd> error) override
    {
        assert(error.size() == x1.size());
        assert(dt > 0 && "dt must be greater then zero!");

        system.dynamics(0.5 * (x1 + x2), u1, error);
        error -= (x2 - x1) / dt;
    }

    void fromParameterServer(const std::string& ns) override{};
};
FACTORY_REGISTER_FD_COLLOCATION(MidpointDiffCollocation)

/**
 * @brief Collocation via Crank-Nicolson differences
 *
 * @ingroup numerics
 *
 * Crank-Nicolson differences approximate \f$ \dot{x} = f(x, u) \f$ in the following manner:
 * \f[
 *    \frac{x_{k+1} - x_k}{\delta T} = 0.5 * ( f(x_k, u_k) + f(x_{k+1}, u_k))
 * \f]
 *
 * @see FiniteDifferencesCollocationInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class CrankNicolsonDiffCollocation : public FiniteDifferencesCollocationInterface
{
 public:
    // Implements interface method
    FiniteDifferencesCollocationInterface::Ptr getInstance() const override { return std::make_shared<CrankNicolsonDiffCollocation>(); }

    // Implements interface method
    void computeEqualityConstraint(const StateVector& x1, const InputVector& u1, const StateVector& x2, double dt,
                                   const SystemDynamicsInterface& system, Eigen::Ref<Eigen::VectorXd> error) override
    {
        assert(error.size() == x1.size());
        assert(dt > 0 && "dt must be greater then zero!");

        Eigen::VectorXd f1(x1.size());
        system.dynamics(x1, u1, f1);
        system.dynamics(x2, u1, error);
        error = (x2 - x1) / dt - 0.5 * (f1 + error);
    }

    void fromParameterServer(const std::string& ns) override{};
};
FACTORY_REGISTER_FD_COLLOCATION(CrankNicolsonDiffCollocation)

}  // namespace mhp_planner

#endif  // SRC_NUMERICS_INCLUDE_MHP_PLANNER_NUMERICS_FINITE_DIFFERENCES_COLLOCATION_H_
