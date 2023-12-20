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

#ifndef SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_STRUCTURED_OCP_DISCRETIZATION_GRIDS_FULL_DISCRETIZATION_GRID_BASE_H_
#define SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_STRUCTURED_OCP_DISCRETIZATION_GRIDS_FULL_DISCRETIZATION_GRID_BASE_H_

#include <mhp_planner/ocp/structured_ocp/discretization_grids/discretization_grid_interface.h>

#include <mhp_planner/hypergraph/graph/scalar_vertex.h>
#include <mhp_planner/hypergraph/graph/vector_vertex.h>

#include <mhp_planner/numerics/finite_differences_collocation.h>

#include <memory>

namespace mhp_planner {

class FullDiscretizationGridBase : public DiscretizationGridInterface
{
 public:
    using Ptr  = std::shared_ptr<FullDiscretizationGridBase>;
    using UPtr = std::unique_ptr<FullDiscretizationGridBase>;

    enum class CostIntegrationRule { LeftSum, TrapezoidalRule };

    FullDiscretizationGridBase()          = default;
    virtual ~FullDiscretizationGridBase() = default;

    //! Return a newly created shared instance of the implemented class
    DiscretizationGridInterface::Ptr getInstance() const override = 0;

    GridUpdateResult update(const Eigen::VectorXd& x0, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref, NlpFunctions& nlp_fun,
                            OptimizationEdgeSet& edges, SystemDynamicsInterface::Ptr dynamics, bool new_run, const Time& t,
                            ReferenceTrajectoryInterface* sref = nullptr, const Eigen::VectorXd* prev_u = nullptr, double prev_u_dt = 0,
                            ReferenceTrajectoryInterface* xinit = nullptr, ReferenceTrajectoryInterface* uinit = nullptr,
                            bool reinit = false) override;

    double getFirstDt() const override { return getDt(); }
    double getFinalTime() const override { return double(getN() - 1) * getDt(); }

    bool hasConstantControls() const override { return true; }
    bool hasSingleDt() const override { return true; }
    bool isTimeVariableGrid() const override { return !isDtFixedIntended(); }
    bool isUniformGrid() const override { return true; }
    bool providesStateTrajectory() const override { return true; }

    bool getFirstControlInput(Eigen::VectorXd& u0) override;

    void getStateAndControlTimeSeries(TimeSeries::Ptr x_sequence, TimeSeries::Ptr u_sequence, double t_max = MHP_PLANNER_INF_DBL) const override;

    void clear() override;

    bool isEmpty() const override { return _x_seq.empty() || _u_seq.empty(); }
    virtual bool isValid() const { return (_x_seq.size() == _u_seq.size()); }

    void setN(int n, bool try_resample = true) override;
    void setInitialDt(double dt) override { setDtRef(dt); }
    double getInitialDt() const override { return getDtRef(); }
    int getInitialN() const override { return _n_ref; }

    int getNRef() const { return _n_ref; }
    int getN() const override { return _x_seq.size() + 1; }
    void setNRef(int n);
    void setDtRef(double dt) { _dt_ref = dt; }
    double getDtRef() const { return _dt_ref; }
    double getDt() const { return _dt.value(); }

    void setXfFixed(const Eigen::Matrix<bool, -1, 1>& xf_fixed)
    {
        _xf_fixed = xf_fixed;
        setModified(true);
    }

    void setFiniteDifferencesCollocationMethod(FiniteDifferencesCollocationInterface::Ptr fd_eval) { _fd_eval = fd_eval; }
    void setCostIntegrationRule(CostIntegrationRule integration) { _cost_integration = integration; }

    const Eigen::VectorXd& getState(int k) const
    {
        assert(k <= getN());
        if (k == _x_seq.size()) return _xf.values();
        return _x_seq[k].values();
    }

    // VertexSet methods
    std::vector<VertexInterface*>& getActiveVertices() override { return _active_vertices; }
    void getVertices(std::vector<VertexInterface*>& vertices) override;  // TODO(roesmann) add dt, just in case

    void fromParameterServer(const std::string& ns) override = 0;

 protected:
    virtual void initializeSequences(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, ReferenceTrajectoryInterface& uref, NlpFunctions& nlp_fun);
    virtual void initializeSequences(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, ReferenceTrajectoryInterface& xref,
                                     ReferenceTrajectoryInterface& uref, NlpFunctions& nlp_fun);
    virtual void warmStartShifting(const Eigen::VectorXd& x0);

    virtual bool adaptGrid(bool new_run, NlpFunctions& nlp_fun) { return false; }  // A subclass might add a grid adaptation, returns true if adapted

    int findNearestState(const Eigen::VectorXd& x0);

    void updateBounds(const NlpFunctions& nlp_fun);

    virtual void resampleTrajectory(int n_new);

    bool checkAndInitializeXfFixedFlags(int dim_x);

    virtual void createEdges(NlpFunctions& nlp_fun, OptimizationEdgeSet& edges, SystemDynamicsInterface::Ptr dynamics) = 0;

    virtual bool isDtFixedIntended() const { return true; }
    virtual bool isGridAdaptActive() const { return false; }

    void computeActiveVertices() override;

    FiniteDifferencesCollocationInterface::Ptr _fd_eval = std::make_shared<CrankNicolsonDiffCollocation>();

    std::vector<VectorVertex> _x_seq;
    std::vector<VectorVertex> _u_seq;
    PartiallyFixedVectorVertex _xf;
    std::vector<VertexInterface*> _active_vertices;

    const NlpFunctions* _nlp_fun = nullptr;  // cache -> for bounds

    int _n_ref     = 11;
    int _n_adapt   = 0;  // if adaption is on and warmstart off, we might use this n instead of n_ref (only if n_adapt > 0)
    double _dt_ref = 0.1;
    ScalarVertex _dt;  // we need a ScalarVertex to use the helper methods in stage_functions.cpp

    // might be required if the last dt should be fixed or if dt is not fixed
    Eigen::Matrix<bool, -1, 1> _xf_fixed;
    double _dt_lb = 0;
    double _dt_ub = MHP_PLANNER_INF_DBL;

    CostIntegrationRule _cost_integration;
};

}  // namespace mhp_planner

#endif  // SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_STRUCTURED_OCP_DISCRETIZATION_GRIDS_FULL_DISCRETIZATION_GRID_BASE_H_
