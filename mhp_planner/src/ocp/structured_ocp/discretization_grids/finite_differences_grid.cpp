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

#include <mhp_planner/ocp/structured_ocp/discretization_grids/finite_differences_grid.h>

#include <mhp_planner/ocp/structured_ocp/edges/finite_differences_collocation_edges.h>

#include <mhp_planner/core/console.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace mhp_planner {

void FiniteDifferencesGrid::createEdges(NlpFunctions& nlp_fun, OptimizationEdgeSet& edges, SystemDynamicsInterface::Ptr dynamics)
{
    assert(isValid());

    // clear edges first
    // TODO(roesmann): we could implement a more efficient strategy without recreating the whole edgeset everytime
    edges.clear();

    int n = getN();

    std::vector<BaseEdge::Ptr> cost_terms, eq_terms, ineq_terms;
    for (int k = 0; k < n - 1; ++k)
    {
        VectorVertex& x_next  = (k < n - 2) ? _x_seq[k + 1] : _xf;
        VectorVertex& u_prev  = (k > 0) ? _u_seq[k - 1] : _u_prev;
        ScalarVertex& dt_prev = (k > 0) ? _dt : _u_prev_dt;

        cost_terms.clear();
        eq_terms.clear();
        ineq_terms.clear();
        nlp_fun.getNonIntegralStageFunctionEdges(k, _x_seq[k], _u_seq[k], _dt, u_prev, dt_prev, cost_terms, eq_terms, ineq_terms);
        for (BaseEdge::Ptr& edge : cost_terms) edges.addObjectiveEdge(edge);
        for (BaseEdge::Ptr& edge : eq_terms) edges.addEqualityEdge(edge);
        for (BaseEdge::Ptr& edge : ineq_terms) edges.addInequalityEdge(edge);

        if (nlp_fun.stage_cost && nlp_fun.stage_cost->hasIntegralTerms(k))
        {
            if (_cost_integration == CostIntegrationRule::TrapezoidalRule)
            {
                TrapezoidalIntegralCostEdge::Ptr edge =
                    std::make_shared<TrapezoidalIntegralCostEdge>(_x_seq[k], _u_seq[k], x_next, _dt, nlp_fun.stage_cost, k);
                edges.addObjectiveEdge(edge);
            }
            else if (_cost_integration == CostIntegrationRule::LeftSum)
            {
                LeftSumCostEdge::Ptr edge = std::make_shared<LeftSumCostEdge>(_x_seq[k], _u_seq[k], _dt, nlp_fun.stage_cost, k);
                edges.addObjectiveEdge(edge);
            }
            else
                PRINT_ERROR_NAMED("Cost integration rule not implemented");
        }

        if (nlp_fun.stage_equalities && nlp_fun.stage_equalities->hasIntegralTerms(k))
        {
            if (_cost_integration == CostIntegrationRule::TrapezoidalRule)
            {
                TrapezoidalIntegralEqualityDynamicsEdge::Ptr edge = std::make_shared<TrapezoidalIntegralEqualityDynamicsEdge>(
                    dynamics, _x_seq[k], _u_seq[k], x_next, _dt, nlp_fun.stage_equalities, k);
                edge->setFiniteDifferencesCollocationMethod(_fd_eval);
                edges.addEqualityEdge(edge);
            }
            else if (_cost_integration == CostIntegrationRule::LeftSum)
            {
                LeftSumEqualityEdge::Ptr edge = std::make_shared<LeftSumEqualityEdge>(_x_seq[k], _u_seq[k], _dt, nlp_fun.stage_equalities, k);
                edges.addEqualityEdge(edge);

                // system dynamics edge
                FDCollocationEdge::Ptr sys_edge = std::make_shared<FDCollocationEdge>(dynamics, _x_seq[k], _u_seq[k], x_next, _dt);
                sys_edge->setFiniteDifferencesCollocationMethod(_fd_eval);
                edges.addEqualityEdge(sys_edge);
            }
            else
                PRINT_ERROR_NAMED("Cost integration rule not implemented");
        }
        else
        {
            // just the system dynamics edge
            FDCollocationEdge::Ptr edge = std::make_shared<FDCollocationEdge>(dynamics, _x_seq[k], _u_seq[k], x_next, _dt);
            edge->setFiniteDifferencesCollocationMethod(_fd_eval);
            edges.addEqualityEdge(edge);
        }

        if (nlp_fun.stage_inequalities && nlp_fun.stage_inequalities->hasIntegralTerms(k))
        {
            if (_cost_integration == CostIntegrationRule::TrapezoidalRule)
            {
                TrapezoidalIntegralInequalityEdge::Ptr edge =
                    std::make_shared<TrapezoidalIntegralInequalityEdge>(_x_seq[k], _u_seq[k], x_next, _dt, nlp_fun.stage_inequalities, k);
                edges.addInequalityEdge(edge);
            }
            else if (_cost_integration == CostIntegrationRule::LeftSum)
            {
                LeftSumInequalityEdge::Ptr edge = std::make_shared<LeftSumInequalityEdge>(_x_seq[k], _u_seq[k], _dt, nlp_fun.stage_inequalities, k);
                edges.addInequalityEdge(edge);
            }
            else
                PRINT_ERROR_NAMED("Cost integration rule not implemented");
        }
    }

    // check if we have a separate unfixed final state
    if (!_xf.isFixed())
    {
        // set final state cost
        BaseEdge::Ptr cost_edge = nlp_fun.getFinalStateCostEdge(n - 1, _xf);
        if (cost_edge) edges.addObjectiveEdge(cost_edge);

        // set final state constraint
        BaseEdge::Ptr constr_edge = nlp_fun.getFinalStateConstraintEdge(n - 1, _xf);
        if (constr_edge)
        {
            if (nlp_fun.final_stage_constraints->isEqualityConstraint())
                edges.addEqualityEdge(constr_edge);
            else
                edges.addInequalityEdge(constr_edge);
        }
    }

    // TODO are there any pratical cases in which the following is actually used?
    // The last control is taken as a control reference and is not part of the actual optimized controls (it is fixed anyway)
    // So what is the purpose of constraining the last actual control (deviation) to a value which we do not care about and is never executed?
    // Since we cannot predict this value reasonably either, we cannot neutralize its negative impacts on all the other controls.

    // add control deviation edges for last control
    // cost_terms.clear();
    // eq_terms.clear();
    // ineq_terms.clear();
    // nlp_fun.getFinalControlDeviationEdges(n, _u_ref, _u_seq.back(), _dt, cost_terms, eq_terms, ineq_terms);
    // for (BaseEdge::Ptr& edge : cost_terms) edges.addObjectiveEdge(edge);
    // for (BaseEdge::Ptr& edge : eq_terms) edges.addEqualityEdge(edge);
    // for (BaseEdge::Ptr& edge : ineq_terms) edges.addInequalityEdge(edge);
}

void FiniteDifferencesGrid::fromParameterServer(const std::string& ns)
{
    ros::NodeHandle nh;

    // Set parameters for finite differences grid
    int n;
    double dt;
    bool warm_start;
    nh.getParam(ns + "/n", n);
    nh.getParam(ns + "/dt", dt);
    nh.getParam(ns + "/warm_start", warm_start);
    if (n < 2) PRINT_ERROR("FiniteDifferencesGrid: Number of states must be greater than or equal 2.");
    if (dt <= 0) PRINT_ERROR("FiniteDifferencesGrid: Dt must be greater than 0.0.");

    setNRef(n);
    setDtRef(dt);
    setWarmStart(warm_start);

    // Set finite differences collocation type
    std::string fd_collocation;
    nh.getParam(ns + "/fd_collocation", fd_collocation);
    FiniteDifferencesCollocationInterface::Ptr fd_eval = create_from_factory<FiniteDifferencesCollocationInterface>(fd_collocation);
    // import parameters
    if (fd_eval)
    {
        fd_eval->fromParameterServer(ns);
        setFiniteDifferencesCollocationMethod(fd_eval);
    }
    else
    {
        PRINT_ERROR("FiniteDifferencesGrid: unknown finite differences collocation method specified.");
        return;
    }

    // Set cost integration rule
    std::string cost_integration_rule;
    nh.getParam(ns + "/cost_integration_rule", cost_integration_rule);
    if (cost_integration_rule == "LeftSum")
        setCostIntegrationRule(CostIntegrationRule::LeftSum);
    else if (cost_integration_rule == "TrapeziodalRule")
        setCostIntegrationRule(CostIntegrationRule::TrapezoidalRule);
    else
        PRINT_ERROR("FiniteDifferencesGrid: Unknown cost integration rule specified.");
}

}  // namespace mhp_planner
