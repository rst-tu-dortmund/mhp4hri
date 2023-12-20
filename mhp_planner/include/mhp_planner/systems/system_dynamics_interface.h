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

#ifndef SRC_SYSTEMS_INCLUDE_MHP_PLANNER_SYSTEMS_SYSTEM_DYNAMICS_INTERFACE_H_
#define SRC_SYSTEMS_INCLUDE_MHP_PLANNER_SYSTEMS_SYSTEM_DYNAMICS_INTERFACE_H_

#include <mhp_planner/core/factory.h>
#include <mhp_planner/core/time.h>
#include <mhp_planner/core/types.h>

#include <ros/ros.h>
#include <memory>

namespace mhp_planner {

class FiniteDifferencesInterface;  // forward declaration

/**
 * @brief Interface class for system dynamic equations
 *
 * @ingroup systems
 *
 * This class specifies methods that are required to be implemented by specific
 * subclasses in order to allow their general utilization in a variety of control tasks.
 *
 * System dynamics can be either defined in continuous-time, e.g.
 * \f[
 *      \dot{x} = f(x, u)
 * \f]
 * or in discrete-time, e.g.
 * \f[
 *      x_{k+1} = f(x_k, u_k)
 * \f].
 * Subclasses must overload isContinuousTime() appropriately.
 *
 * @remark This interface is provided with factory support (SystemDynamicsFactory).
 *
 * @see SystemOutputInterface PlantInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class SystemDynamicsInterface
{
 public:
    using Ptr           = std::shared_ptr<SystemDynamicsInterface>;
    using StateVector   = Eigen::VectorXd;
    using ControlVector = Eigen::VectorXd;

    //! Default constructor
    SystemDynamicsInterface();

    //! Default destructor
    virtual ~SystemDynamicsInterface() = default;

    //! Return a newly created shared instance of the implemented class
    virtual Ptr getInstance() const = 0;

    virtual bool isContinuousTime() const = 0;

    virtual bool isLinear() const = 0;

    //! Return the plant input dimension (u)
    virtual int getInputDimension() const = 0;
    //! Return state dimension (x)
    virtual int getStateDimension() const = 0;

    virtual void dynamics(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u, Eigen::Ref<StateVector> f) const = 0;

    virtual void getLinearA(const StateVector& x0, const ControlVector& u0, Eigen::MatrixXd& A) const;

    virtual void getLinearB(const StateVector& x0, const ControlVector& u0, Eigen::MatrixXd& B) const;

    void setLinearizationMethod(std::shared_ptr<FiniteDifferencesInterface> lin_method);

    virtual void reset() {}

    virtual void fromParameterServer(const std::string& ns) = 0;

 private:
    std::shared_ptr<FiniteDifferencesInterface> _linearization_method;
};

using SystemDynamicsFactory = Factory<SystemDynamicsInterface>;
#define FACTORY_REGISTER_SYSTEM_DYNAMICS(type) FACTORY_REGISTER_OBJECT(type, SystemDynamicsInterface)

class ParallelIntegratorSystem : public SystemDynamicsInterface
{
 public:
    //! Default constructor (do not forget to set the dimension)
    ParallelIntegratorSystem() {}
    //! Construct ingerator system with given order/dimension
    explicit ParallelIntegratorSystem(int dimension) : _dimension(dimension) {}

    // implements interface method
    Ptr getInstance() const override { return std::make_shared<ParallelIntegratorSystem>(); }

    // implements interface method
    bool isContinuousTime() const override { return true; }
    // implements interface method
    bool isLinear() const override { return true; }

    // implements interface method
    int getInputDimension() const override { return _dimension; }
    // implements interface method
    int getStateDimension() const override { return _dimension; }

    // implements interface method
    void dynamics(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u, Eigen::Ref<StateVector> f) const override
    {
        assert(x.rows() == _dimension);
        assert(x.rows() == f.rows() && "ParallelIntegratorSystem::dynamics(): x and f are not of the same size, do not forget to pre-allocate f.");

        f = _time_constant * u;
    }

    // access parameters

    //! Get current integrator chain dimension / order of the system
    const int& getDimension() const { return _dimension; }
    //! Set integrator dimension (p >= 1)
    void setDimension(int dim) { _dimension = dim; }
    //! Get time constant T of the integrator
    const double& getTimeConstant() const { return _time_constant; }
    //! Set Time constant T of the integrator
    void setTimeConstant(double time_constant) { _time_constant = time_constant; }

    void fromParameterServer(const std::string& ns) override
    {
        SystemDynamicsInterface::fromParameterServer(ns);

        ros::NodeHandle nh;

        nh.getParam(ns + "/dimension", _dimension);
        nh.getParam(ns + "/time_constant", _time_constant);
    }

 private:
    int _dimension        = 1;
    double _time_constant = 1.0;
};
FACTORY_REGISTER_SYSTEM_DYNAMICS(ParallelIntegratorSystem)

}  // namespace mhp_planner

#endif  // SRC_SYSTEMS_INCLUDE_MHP_PLANNER_SYSTEMS_SYSTEM_DYNAMICS_INTERFACE_H_
