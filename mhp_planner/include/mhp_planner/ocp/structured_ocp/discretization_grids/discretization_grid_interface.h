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

#ifndef SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_STRUCTURED_OCP_DISCRETIZATION_GRIDS_DISCRETIZATION_GRID_INTERFACE_H_
#define SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_STRUCTURED_OCP_DISCRETIZATION_GRIDS_DISCRETIZATION_GRID_INTERFACE_H_

#include <mhp_planner/hypergraph/graph/edge_set.h>
#include <mhp_planner/hypergraph/graph/vertex_set.h>

#include <mhp_planner/ocp/functions/nlp_functions.h>

#include <mhp_planner/systems/system_dynamics_interface.h>

#include <mhp_planner/core/factory.h>
#include <mhp_planner/core/reference_trajectory.h>
#include <mhp_planner/core/time_series.h>
#include <mhp_planner/core/types.h>

#include <memory>
#include <utility>

namespace mhp_planner {

struct GridUpdateResult
{
    bool vertices_updated = false;
    bool edges_updated    = false;

    bool updated() const { return vertices_updated || edges_updated; }
};

/**
 * @brief Generic interface class for discretization grids
 *
 * @ingroup optimization discretization-grid
 *
 * This abstract class defines the interface for discretization grids.
 * Discretization grids represent state and control sequences subject to
 * optimziation.
 * Usually, in direct optimal control, continuous-time trajectories are discretized
 * in order to transform the optimization problem into a finite parameter
 * nonlinear optimization problem, also called nonlinear program.
 *
 * A discretization grid directly interacts with a Hyper-Graph to define the
 * optimal control problem. Hence, the grid provides the relevant vertices
 * (optimization parameters) and the corresponding edge creation.
 *
 * @remark This interface is provided with factory support (DiscretizationGridFactory).
 *
 * @see HyperGraph
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class DiscretizationGridInterface : public VertexSetInterface
{
 public:
    using Ptr  = std::shared_ptr<DiscretizationGridInterface>;
    using UPtr = std::unique_ptr<DiscretizationGridInterface>;

    DiscretizationGridInterface() = default;
    DiscretizationGridInterface(int state_dim, int control_dim);

    //! Virtual destructor
    virtual ~DiscretizationGridInterface() = default;

    // TODO(roesmann): detailed description here (very important)
    // nlp_fun->update must be called inside ...
    virtual GridUpdateResult update(const Eigen::VectorXd& x0, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref,
                                    NlpFunctions& nlp_fun, OptimizationEdgeSet& edges, SystemDynamicsInterface::Ptr dynamics, bool new_run,
                                    const Time& t, ReferenceTrajectoryInterface* sref = nullptr, const Eigen::VectorXd* prev_u = nullptr,
                                    double prev_u_dt = 0, ReferenceTrajectoryInterface* xinit = nullptr,
                                    ReferenceTrajectoryInterface* uinit = nullptr, bool reinit = false) = 0;

    //! Return a newly created shared instance of the implemented class
    virtual Ptr getInstance() const = 0;

    //! Get access to the accociated factory
    static Factory<DiscretizationGridInterface>& getFactory() { return Factory<DiscretizationGridInterface>::instance(); }

    //! Return dimension of the control input dimension in the grid
    // int getControlInputDimension() const { return _control_dim; }
    // //! Return dimension of the state dimension in the grid
    // int getStateDimension() const { return _state_dim; }

    virtual void setN(int n, bool try_resample = true) = 0;
    virtual void setInitialDt(double dt)               = 0;

    virtual void setWarmStart(bool active) { _warm_start = active; }
    virtual bool getWarmStart() { return _warm_start; }

    virtual int getN() const            = 0;
    virtual double getFirstDt() const   = 0;
    virtual double getFinalTime() const = 0;
    virtual double getInitialDt() const = 0;
    virtual int getInitialN() const     = 0;

    // TODO(roesmann): check what we really need:
    virtual bool hasConstantControls() const     = 0;
    virtual bool hasSingleDt() const             = 0;
    virtual bool isTimeVariableGrid() const      = 0;
    virtual bool isUniformGrid() const           = 0;
    virtual bool providesStateTrajectory() const = 0;

    // TODO(roesmann): see notes in the optimal_control_problem_interface.h
    //! Return state and control trajectory as time series object (shared instance)
    virtual void getStateAndControlTimeSeries(TimeSeries::Ptr x_sequence, TimeSeries::Ptr u_sequence, double t_max = MHP_PLANNER_INF_DBL) const = 0;

    virtual bool getFirstControlInput(Eigen::VectorXd& u0) = 0;

    void clear() override = 0;

    virtual bool isEmpty() const = 0;

    // void printGrid();
    virtual void fromParameterServer(const std::string& ns) = 0;

 protected:
    // VertexSet methods
    std::vector<VertexInterface*>& getActiveVertices() override        = 0;
    void getVertices(std::vector<VertexInterface*>& vertices) override = 0;
    // compute active vertices: override in subclass to be more efficient
    void computeActiveVertices() override = 0;

    void setPreviousControl(const Eigen::VectorXd& prev_u, double prev_u_dt)
    {
        _u_prev.values() = prev_u;
        _u_prev.setFixed(true);
        _u_prev_dt.value() = prev_u_dt;
        _u_prev_dt.setFixed(true);
    }
    void setLastControlRef(const Eigen::VectorXd& last_u_ref)
    {
        _u_ref.values() = last_u_ref;
        _u_ref.setFixed(true);
    }

    VectorVertex _u_prev;  // TODO(roesmann): how should we handle this?
    ScalarVertex _u_prev_dt;
    VectorVertex _u_ref;  // for last control deviation if desired

    bool _warm_start = false;
    bool _first_run  = true;
};

using DiscretizationGridFactory = Factory<DiscretizationGridInterface>;
#define FACTORY_REGISTER_DISCRETIZATION_GRID(type) FACTORY_REGISTER_OBJECT(type, DiscretizationGridInterface)

}  // namespace mhp_planner

#endif  // SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_STRUCTURED_OCP_DISCRETIZATION_GRIDS_DISCRETIZATION_GRID_INTERFACE_H_
