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

#include <mhp_planner/ocp/structured_ocp/structured_optimal_control_problem.h>

#include <mhp_planner/ocp/structured_ocp/discretization_grids/discretization_grid_interface.h>

#include <memory>

namespace mhp_planner {

StructuredOptimalControlProblem::StructuredOptimalControlProblem() {}

StructuredOptimalControlProblem::StructuredOptimalControlProblem(DiscretizationGridInterface::Ptr grid, SystemDynamicsInterface::Ptr dynamics,
                                                                 BaseHyperGraphOptimizationProblem::Ptr optim_prob, NlpSolverInterface::Ptr solver)
    : _grid(grid), _optim_prob(optim_prob), _dynamics(dynamics), _solver(solver)
{
    _optim_prob->setGraph(_edges, _grid);
}

int StructuredOptimalControlProblem::getPerformedIterations() const { return _iterations; }

bool StructuredOptimalControlProblem::providesFutureControls() const { return true; }

bool StructuredOptimalControlProblem::providesFutureStates() const { return _grid ? _grid->providesStateTrajectory() : false; }

bool StructuredOptimalControlProblem::initialize(const int id)
{
    if (!_optim_prob)
    {
        PRINT_ERROR(
            "StructuredOptimalControlProblem::initialize(): no hyper-graph "
            "optimization problem strategy specified.");
        return false;
    }
    if (!_solver || !_solver->initialize(_optim_prob.get()))
    {
        PRINT_ERROR(
            "StructuredOptimalControlProblem::initialize(): no solver "
            "specified or solver initialization failed.");
        return false;
    }

    if (_u_prev.size() == 0)
    {
        // Default setting for previous control and dt if nothing specified
        _u_prev.setZero(_dynamics->getInputDimension());
        _u_prev_dt = _grid->getInitialDt();
    }

    return true;
}

bool StructuredOptimalControlProblem::compute(const StateVector& x, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref,
                                              ReferenceTrajectoryInterface* sref, const Time& t, bool new_run, ReferenceTrajectoryInterface* xinit,
                                              ReferenceTrajectoryInterface* uinit, const std::string& ns, bool reinit)
{
    if (!_grid)
    {
        PRINT_ERROR(
            "StructuredOptimalControlProblem::compute(): no discretization "
            "grid specified.");
        return false;
    }

    if (!_dynamics)
    {
        PRINT_ERROR(
            "StructuredOptimalControlProblem::compute(): no system "
            "dynamics model specified.");
        return false;
    }
    if (!_optim_prob)
    {
        PRINT_ERROR(
            "StructuredOptimalControlProblem::compute(): no hyper-graph "
            "optimization strategy specified.");
        return false;
    }
    if (!_solver)
    {
        PRINT_ERROR("StructuredOptimalControlProblem::compute(): no solver specified.");
        return false;
    }
    if (!_functions.stage_cost && !_functions.final_stage_cost)
    {
        PRINT_WARNING(
            "StructuredOptimalControlProblem::compute(): no cost "
            "function specified.");
    }

    bool success = false;

    // reset ts caches
    _ts_x_cache.reset();
    _ts_u_cache.reset();
    _ts_dt_cache = 0;

    if (_statistics) _statistics->clear();

    Time t1 = Time::now();

    GridUpdateResult grid_udpate_result =
        _grid->update(x, xref, uref, _functions, *_edges, _dynamics, new_run, t, sref, &_u_prev, _u_prev_dt, xinit, uinit, reinit);

    if (grid_udpate_result.vertices_updated)
    {
        _optim_prob->precomputeVertexQuantities();
    }
    if (grid_udpate_result.updated())
    {
        _optim_prob->precomputeEdgeQuantities();
    }

    assert(_optim_prob->getGraph().checkGraphConsistency());

    Time t2 = Time::now();

    // StageFunctionCounter::setZero();
    // StageFunctionCounter::print();

    SolverStatus status = _solver->solve(*_optim_prob, grid_udpate_result.updated(), new_run, _grid->getWarmStart(), &_objective_value, &_iterations);

    if (status == SolverStatus::Converged || status == SolverStatus::EarlyTerminated)
    {
        success = true;
    }
    if (status == SolverStatus::Infeasible)
        PRINT_ERROR(
            "StructuredOptimalControlProblem::compute(): infeasible "
            "problem detected.");
    if (status == SolverStatus::Error) PRINT_ERROR("StructuredOptimalControlProblem::compute(): solver error detected.");

    // StageFunctionCounter::print();

    Time t3 = Time::now();

    if (_statistics)
    {
        _statistics->preparation_time = t2 - t1;
        _statistics->solving_time     = t3 - t2;
    }

    // PRINT_INFO("CPU time of only the solving phase: " << (t2 - t1).toSec() *
    // 1000.0 << " ms.");

    return success;
}

bool StructuredOptimalControlProblem::getFirstControlInput(ControlVector& u0) const
{
    if (!_grid) return false;
    if (_grid->getFirstControlInput(u0))
        return true;
    else
        return false;
}

void StructuredOptimalControlProblem::setBounds(const Eigen::VectorXd& x_lb, const Eigen::VectorXd& x_ub, const Eigen::VectorXd& u_lb,
                                                const Eigen::VectorXd& u_ub)
{
    _functions.x_lb = x_lb;
    _functions.x_ub = x_ub;
    _functions.u_lb = u_lb;
    _functions.u_ub = u_ub;
    // TODO(roesmann): ocp modified?! we just changed bounds and not dimensions
}

bool StructuredOptimalControlProblem::getBounds(Eigen::Ref<Eigen::Vector<double, 6>> x_lb, Eigen::Ref<Eigen::Vector<double, 6>> x_ub,
                                                Eigen::Ref<Eigen::Vector<double, 6>> u_lb, Eigen::Ref<Eigen::Vector<double, 6>> u_ub) const
{
    x_lb = _functions.x_lb;
    x_ub = _functions.x_ub;
    u_lb = _functions.u_lb;
    u_ub = _functions.u_ub;

    return true;
}

void StructuredOptimalControlProblem::setStateBounds(const Eigen::VectorXd& x_lb, const Eigen::VectorXd& x_ub)
{
    _functions.x_lb = x_lb;
    _functions.x_ub = x_ub;
    // TODO(roesmann): ocp modified?! we just changed bounds and not dimensions
}

void StructuredOptimalControlProblem::setControlBounds(const Eigen::VectorXd& u_lb, const Eigen::VectorXd& u_ub)
{
    _functions.u_lb = u_lb;
    _functions.u_ub = u_ub;
    // TODO(roesmann): ocp modified?! we just changed bounds and not dimensions
}

void StructuredOptimalControlProblem::getTimeSeries(TimeSeries::Ptr x_sequence, TimeSeries::Ptr u_sequence, double t_max)
{
    if (!_grid)
    {
        PRINT_ERROR_NAMED("No grid loaded.");
        return;
    }
    _grid->getStateAndControlTimeSeries(x_sequence, u_sequence);
}

void StructuredOptimalControlProblem::fromParameterServer(const std::string& ns)
{
    // Reset class members and class
    _dynamics.reset();
    _solver.reset();
    _grid.reset();
    _optim_prob.reset();

    reset();

    ros::NodeHandle nh;

    // Set system dynamics
    std::string dynamics_type;
    nh.getParam(ns + "/system_dynamics/dynamics_type", dynamics_type);
    SystemDynamicsInterface::Ptr system_dynamics = SystemDynamicsFactory::instance().create(dynamics_type);
    system_dynamics->fromParameterServer(ns + "/system_dynamics");
    setSystemDynamics(system_dynamics);

    // Set discretizaion grids
    DiscretizationGridInterface::Ptr grid = DiscretizationGridFactory::instance().create("FiniteDifferencesGrid");
    grid->fromParameterServer(ns + "/finite_differences_grid");
    setDiscretizationGrid(grid);

    // Set stage cost
    std::string stage_cost_type;
    nh.getParam(ns + "/stage_cost/stage_cost_type", stage_cost_type);

    if (stage_cost_type == "None")
    {
        PRINT_WARNING("StructuredOptimalControlProblem: no stage cost specified.");
        setStageCost({});
    }
    else if (stage_cost_type == "URStageCost")
    {
        StageCost::Ptr stage_cost = StageCostFactory::instance().create(stage_cost_type);
        if (stage_cost)
        {
            if (!stage_cost->fromParameterServer(ns + "/stage_cost/ur_stage_cost"))
            {
                PRINT_ERROR("StructuredOptimalControlProblem: could not load stage cost.");
                return;
            }
            if (!stage_cost->checkParameters(_dynamics->getStateDimension(), _dynamics->getInputDimension()))
            {
                PRINT_ERROR(
                    "StructuredOptimalControlProblem: could not verify stage "
                    "cost dimensions.");
                return;
            }
            setStageCost(stage_cost);
        }
        else
        {
            PRINT_ERROR("StructuredOptimalControlProblem: could not create stage cost.");
            return;
        }
    }
    else
    {
        PRINT_ERROR("StructuredOptimalControlProblem: stage cost type not implemented.");
        return;
    }

    // Set final stage cost
    std::string final_stage_cost_type;
    nh.getParam(ns + "/final_stage_cost/final_stage_cost_type", final_stage_cost_type);

    if (final_stage_cost_type == "None")
    {
        PRINT_WARNING("StructuredOptimalControlProblem: no final stage cost specified.");
        setFinalStageCost({});
    }
    else if (final_stage_cost_type == "URFinalStageCost")
    {
        FinalStageCost::Ptr final_stage_cost = FinalStageCostFactory::instance().create(final_stage_cost_type);
        if (final_stage_cost)
        {
            if (!final_stage_cost->fromParameterServer(ns + "/final_stage_cost/ur_final_stage_cost"))
            {
                PRINT_ERROR(
                    "StructuredOptimalControlProblem: could not load final "
                    "stage cost.");
                return;
            }
            if (!final_stage_cost->checkParameters(_dynamics->getStateDimension(), _dynamics->getInputDimension()))
            {
                PRINT_ERROR(
                    "StructuredOptimalControlProblem: could not verify final "
                    "stage cost dimensions.");
                return;
            }
            setFinalStageCost(final_stage_cost);
        }
        else
        {
            PRINT_ERROR(
                "StructuredOptimalControlProblem: could not create final "
                "stage cost.");
            return;
        }
    }
    else
    {
        PRINT_ERROR(
            "StructuredOptimalControlProblem: final stage cost type not "
            "implemented.");
        return;
    }

    // Set stage equalities
    std::string stage_equalities;
    nh.getParam(ns + "/stage_equalities/stage_equalities_type", stage_equalities);
    if (stage_equalities == "None")
    {
        PRINT_INFO("StructuredOptimalControlProblem: no stage equalities specified.");
    }
    else
    {
        PRINT_ERROR(
            "StructuredOptimalControlProblem: stage equalities not "
            "implemented yet.");
        return;
    }

    // Set stage inequalities
    std::string stage_inequalities;
    nh.getParam(ns + "/stage_inequalities/stage_inequalities_type", stage_inequalities);
    if (stage_inequalities == "None")
    {
        PRINT_INFO("StructuredOptimalControlProblem: no stage inequalities specified.");
    }
    else if (stage_inequalities == "URInequalityConstraint")
    {
        StageInequalityConstraint::Ptr stage_inequality_constraint = StageInequalitiesFactory::instance().create(stage_inequalities);
        if (stage_inequality_constraint)
        {
            if (!stage_inequality_constraint->fromParameterServer(ns + "/stage_inequalities/ur_inequality_constraint"))
            {
                PRINT_ERROR(
                    "StructuredOptimalControlProblem: could not load stage "
                    "inequalitites.");
                return;
            }
            if (!stage_inequality_constraint->checkParameters(_dynamics->getStateDimension(), _dynamics->getInputDimension()))
            {
                PRINT_ERROR(
                    "StructuredOptimalControlProblem: could not verify stage "
                    "inequalitites dimensions.");
                return;
            }
            setStageInequalityConstraint(stage_inequality_constraint);
        }
        else
        {
            PRINT_ERROR(
                "StructuredOptimalControlProblem: could not create stage "
                "inequalitites.");
            return;
        }
    }
    else
    {
        PRINT_ERROR(
            "StructuredOptimalControlProblem: stage inequalitites not "
            "implemented yet.");
        return;
    }

    // Set final stage constraints
    std::string final_stage_constraints;
    nh.getParam(ns + "/final_stage_constraints/final_stage_constraints_type", final_stage_constraints);

    if (final_stage_constraints == "None")
    {
        PRINT_INFO(
            "StructuredOptimalControlProblem: no final stage constraints "
            "specified.");
    }
    else if (final_stage_constraints == "URFinalStateConstraintJointSpace")
    {
        FinalStageConstraint::Ptr final_stage_inequality_constraint = FinalStageConstraintFactory::instance().create(final_stage_constraints);
        if (final_stage_inequality_constraint)
        {
            if (!final_stage_inequality_constraint->fromParameterServer(ns + "/final_stage_constraints/"
                                                                             "ur_final_state_constraint_joint_space"))
            {
                PRINT_ERROR(
                    "StructuredOptimalControlProblem: could not load final "
                    "stage constraint.");
                return;
            }
            if (!final_stage_inequality_constraint->checkParameters(_dynamics->getStateDimension(), _dynamics->getInputDimension(),
                                                                    _functions.final_stage_cost))
            {
                PRINT_ERROR(
                    "StructuredOptimalControlProblem: could not verify final "
                    "stage constraint dimensions.");
                return;
            }
            setFinalStageConstraint(final_stage_inequality_constraint);
        }
        else
        {
            PRINT_ERROR(
                "StructuredOptimalControlProblem: could not create final "
                "stage constraints.");
            return;
        }
    }
    else
    {
        PRINT_ERROR(
            "StructuredOptimalControlProblem: final stage constraints not "
            "implemented yet.");
        return;
    }

    // Set stage preprocessor
    std::string stage_preprocessor_type;
    nh.getParam(ns + "/stage_preprocessor/stage_preprocessor_type", stage_preprocessor_type);
    if (stage_preprocessor_type == "None")
    {
        PRINT_INFO("StructuredOptimalControlProblem: no stage preprocessor specified.");
        setStagePreprocessor({});
    }
    else if (stage_preprocessor_type == "URStagePreprocessor")
    {
        StagePreprocessor::Ptr stage_preprocessor = StagePreprocessorFactory::instance().create(stage_preprocessor_type);

        if (stage_preprocessor)
        {
            if (!stage_preprocessor->fromParameterServer(ns + "/stage_preprocessor/ur_stage_preprocessor"))
            {
                PRINT_ERROR(
                    "StructuredOptimalControlProblem: could not load stage "
                    "preprocessor.");
                return;
            }
            setStagePreprocessor(stage_preprocessor);
        }
        else
        {
            PRINT_ERROR(
                "StructuredOptimalControlProblem: could not create stage "
                "preprocessor.");
            return;
        }
    }
    else
    {
        PRINT_ERROR(
            "StructuredOptimalControlProblem: stage preprocessor not "
            "implemented yet.");
        return;
    }

    // Set state and control boundaries
    std::vector<double> x_lb;
    nh.getParam(ns + "/x_lb", x_lb);
    Eigen::VectorXd x_lb_eigen = Eigen::Map<Eigen::VectorXd>(x_lb.data(), x_lb.size());
    std::vector<double> x_ub;
    nh.getParam(ns + "/x_ub", x_ub);
    Eigen::VectorXd x_ub_eigen = Eigen::Map<Eigen::VectorXd>(x_ub.data(), x_ub.size());
    std::vector<double> u_lb;
    nh.getParam(ns + "/u_lb", u_lb);
    Eigen::VectorXd u_lb_eigen = Eigen::Map<Eigen::VectorXd>(u_lb.data(), u_lb.size());
    std::vector<double> u_ub;
    nh.getParam(ns + "/u_ub", u_ub);
    Eigen::VectorXd u_ub_eigen = Eigen::Map<Eigen::VectorXd>(u_ub.data(), u_ub.size());
    setBounds(x_lb_eigen, x_ub_eigen, u_lb_eigen, u_ub_eigen);

    // Set Hypergraph Optimization Problem as vertex based
    BaseHyperGraphOptimizationProblem::Ptr optim_prob =
        HyperGraphOptimizationProblemFactory::instance().create("HyperGraphOptimizationProblemVertexBased");
    optim_prob->fromParameterServer(ns + "/HyperGraphOptimizationProblemVertexBased");
    setHyperGraphOptimizationProblem(optim_prob);

    // Set NLP solver
    NlpSolverInterface::Ptr nlp_solver = NlpSolverFactory::instance().create("SolverIpopt");
    if (nlp_solver)
    {
        if (nlp_solver->fromParameterServer(ns + "/ipopt"))
        {
            setSolver(nlp_solver);
        }
        else
        {
            PRINT_ERROR("StructuredOptimalControlProblem: could not load IPOPT solver.");
            return;
        }
    }
    else
    {
        PRINT_ERROR("StructuredOptimalControlProblem: could not create IPOPT solver.");
        return;
    }
}

void StructuredOptimalControlProblem::reset()
{
    if (_grid) _grid->clear();
    if (_optim_prob) _optim_prob->clear();
    if (_dynamics) _dynamics->reset();
    if (_solver) _solver->clear();
    if (_statistics) _statistics->clear();

    _u_prev.setZero();  // TODO(roesmann): should we set this to zero here?

    _ocp_modified    = true;
    _objective_value = -1;
}

}  // namespace mhp_planner
