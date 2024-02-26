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

#ifndef SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_STRUCTURED_OCP_STRUCTURED_OPTIMAL_CONTROL_PROBLEM_H_
#define SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_STRUCTURED_OCP_STRUCTURED_OPTIMAL_CONTROL_PROBLEM_H_

#include <mhp_planner/ocp/optimal_control_problem_interface.h>

#include <mhp_planner/hypergraph/graph/hyper_graph_optimization_problem_base.h>
#include <mhp_planner/ocp/functions/nlp_functions.h>
#include <mhp_planner/ocp/structured_ocp/discretization_grids/discretization_grid_interface.h>
#include <mhp_planner/solvers/nlp_solver_interface.h>
#include <mhp_planner/systems/system_dynamics_interface.h>

#include <ros/ros.h>
#include <memory>
namespace mhp_planner {

class StructuredOptimalControlProblem : public OptimalControlProblemInterface
{
 public:
    using Ptr           = std::shared_ptr<StructuredOptimalControlProblem>;
    using UPtr          = std::unique_ptr<StructuredOptimalControlProblem>;
    using StateVector   = Eigen::VectorXd;
    using ControlVector = Eigen::VectorXd;

    StructuredOptimalControlProblem();
    StructuredOptimalControlProblem(DiscretizationGridInterface::Ptr grid, SystemDynamicsInterface::Ptr dynamics,
                                    BaseHyperGraphOptimizationProblem::Ptr optim_prob, NlpSolverInterface::Ptr solver);

    virtual ~StructuredOptimalControlProblem() {}

    OptimalControlProblemInterface::Ptr getInstance() const override { return std::make_shared<StructuredOptimalControlProblem>(); }

    int getControlInputDimension() const override { return _dynamics ? _dynamics->getInputDimension() : 0; }
    int getStateDimension() const override { return _dynamics ? _dynamics->getStateDimension() : 0; }
    int getN() const override { return _grid ? _grid->getN() : 0; }
    int getPerformedIterations() const override;
    double getFirstDt() const override { return _grid ? _grid->getFirstDt() : 0.0; }
    bool isConstantControlAction() const override { return _grid ? _grid->hasConstantControls() : true; }

    bool providesFutureControls() const override;
    bool providesFutureStates() const override;

    bool initialize(const int id) override;

    bool compute(const StateVector& x, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref, ReferenceTrajectoryInterface* sref,
                 const Time& t, bool new_run, ReferenceTrajectoryInterface* xinit = nullptr, ReferenceTrajectoryInterface* uinit = nullptr,
                 const std::string& ns = "", bool reinit = false) override;

    bool getFirstControlInput(ControlVector& u0) const override;

    void setPreviousControlInput(const Eigen::Ref<const ControlVector>& u_prev, double dt) override
    {
        _u_prev = u_prev;
        setPreviousControlInputDt(dt);
    }

    void setPreviousControlInputDt(double dt) override { _u_prev_dt = dt; }

    void setBounds(const Eigen::VectorXd& x_lb, const Eigen::VectorXd& x_ub, const Eigen::VectorXd& u_lb, const Eigen::VectorXd& u_ub);
    bool getBounds(Eigen::Ref<Eigen::Vector<double, 6>> x_lb, Eigen::Ref<Eigen::Vector<double, 6>> x_ub, Eigen::Ref<Eigen::Vector<double, 6>> u_lb,
                   Eigen::Ref<Eigen::Vector<double, 6>> u_ub) const override;

    void setStateBounds(const Eigen::VectorXd& x_lb, const Eigen::VectorXd& x_ub);
    void setControlBounds(const Eigen::VectorXd& u_lb, const Eigen::VectorXd& u_ub);

    void setStageCost(StageCost::Ptr stage_cost)
    {
        _ocp_modified         = true;
        _functions.stage_cost = stage_cost;
    }

    void setFinalStageCost(FinalStageCost::Ptr final_stage_cost)
    {
        _ocp_modified               = true;
        _functions.final_stage_cost = final_stage_cost;
    }

    void setStageEqualityConstraint(StageEqualityConstraint::Ptr stage_eq)
    {
        _ocp_modified               = true;
        _functions.stage_equalities = stage_eq;
    }

    void setStageInequalityConstraint(StageInequalityConstraint::Ptr stage_ineq)
    {
        _ocp_modified                 = true;
        _functions.stage_inequalities = stage_ineq;
    }

    void setFinalStageConstraint(FinalStageConstraint::Ptr final_stage_constraint)
    {
        _ocp_modified                      = true;
        _functions.final_stage_constraints = final_stage_constraint;
    }

    void setStagePreprocessor(StagePreprocessor::Ptr stage_preprocessor)
    {
        _ocp_modified                 = true;
        _functions.stage_preprocessor = stage_preprocessor;
    }

    void setDiscretizationGrid(DiscretizationGridInterface::Ptr grid)
    {
        reset();
        _grid = grid;
        if (_optim_prob) _optim_prob->setGraph(_edges, _grid);
    }
    DiscretizationGridInterface::Ptr getDiscretizationGrid() { return _grid; }

    void setSystemDynamics(SystemDynamicsInterface::Ptr dynamics) { _dynamics = dynamics; }
    void setHyperGraphOptimizationProblem(BaseHyperGraphOptimizationProblem::Ptr optim_prob)
    {
        reset();
        _optim_prob = optim_prob;
        if (_edges && _grid) _optim_prob->setGraph(_edges, _grid);
    }
    void setSolver(NlpSolverInterface::Ptr solver)
    {
        reset();
        _solver = solver;
    }

    void setOptimizedTimeSeriesDt(double dt) { _resample_dt_hint = dt; }  // TODO(roesmann): check if needed
    void setOptimizedTimeSeriesTf(double tf) { _resample_tf = tf; }       // TODO(roesmann): check if needed

    void setStatisticsObject(OptimalControlProblemStatistics::Ptr statistics) { _statistics = statistics; }
    OptimalControlProblemStatistics::Ptr getStatistics() const override { return _statistics; }

    void getTimeSeries(TimeSeries::Ptr x_sequence, TimeSeries::Ptr u_sequence, double t_max = MHP_PLANNER_INF_DBL) override;

    double getCurrentObjectiveValue() override { return _objective_value; }

    const NlpFunctions& getNlpFunctions() const { return _functions; }

    void fromParameterServer(const std::string& ns) override;

    void reset() override;

 protected:
    NlpFunctions _functions;
    DiscretizationGridInterface::Ptr _grid;
    OptimizationEdgeSet::Ptr _edges = std::make_shared<OptimizationEdgeSet>();
    BaseHyperGraphOptimizationProblem::Ptr _optim_prob;
    SystemDynamicsInterface::Ptr _dynamics;
    NlpSolverInterface::Ptr _solver;
    OptimalControlProblemStatistics::Ptr _statistics;

    double _objective_value = -1;
    int _iterations         = -1;

    ControlVector _u_prev;  // we cache the last commanded control
    double _u_prev_dt = 0;  // time stamp of _u_prev

    TimeSeries::Ptr _ts_u_cache;  // TODO(roesmann): check if needed
    TimeSeries::Ptr _ts_x_cache;  // TODO(roesmann): check if needed
    double _ts_dt_cache = 0;      // TODO(roesmann): check if needed

    bool _ocp_modified = true;  // TODO(roesmann): are we really using this right now? Do we need this?

    double _resample_dt_hint = -1;  // TODO(roesmann): check if needed
    double _resample_tf      = -1;  // TODO(roesmann): check if needed

    bool _increase_n_if_infeas = false;
};

FACTORY_REGISTER_OCP(StructuredOptimalControlProblem)

}  // namespace mhp_planner

#endif  // SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_STRUCTURED_OCP_STRUCTURED_OPTIMAL_CONTROL_PROBLEM_H_
