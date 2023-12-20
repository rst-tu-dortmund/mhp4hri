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

#ifndef SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_FUNCTIONS_NLP_FUNCTIONS_H_
#define SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_FUNCTIONS_NLP_FUNCTIONS_H_

#include <mhp_planner/ocp/functions/stage_functions.h>

#include <memory>

namespace mhp_planner {

class DiscretizationGridInterface;

class NlpFunctions
{
 public:
    StageCost::Ptr stage_cost;
    FinalStageCost::Ptr final_stage_cost;
    StageEqualityConstraint::Ptr stage_equalities;
    StageInequalityConstraint::Ptr stage_inequalities;
    FinalStageConstraint::Ptr final_stage_constraints;
    StagePreprocessor::Ptr stage_preprocessor;

    Eigen::VectorXd x_lb;
    Eigen::VectorXd x_ub;
    Eigen::VectorXd u_lb;
    Eigen::VectorXd u_ub;

    // TODO(roesmann): do we really need sref here? can we add that to the particular cost functions that require it?
    // TODO(roesmann): Actually, I don't like the single_dt information here. Is there a more abstract way? Uniform grid?
    bool update(int n, double t, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref, ReferenceTrajectoryInterface* sref,
                bool single_dt, const Eigen::VectorXd& x0, const std::vector<double>& dts, const DiscretizationGridInterface* grid);

    // TODO(roesmann) this complete nlp function vs edges stuff could definitely need a redesign...
    void getNonIntegralStageFunctionEdges(int k, VectorVertex& xk, VectorVertex& uk, ScalarVertex& dt, VectorVertex& u_prev, ScalarVertex& u_prev_dt,
                                          const StageFunction& stage_fun, std::vector<BaseEdge::Ptr>& edges);

    void getNonIntegralStageFunctionEdges(int k, VectorVertex& xk, VectorVertex& uk, ScalarVertex& dt, VectorVertex& u_prev, ScalarVertex& u_prev_dt,
                                          std::vector<BaseEdge::Ptr>& cost_edges, std::vector<BaseEdge::Ptr>& eq_edges,
                                          std::vector<BaseEdge::Ptr>& ineq_edges);

    void getFinalControlDeviationEdges(int n, VectorVertex& u_ref, VectorVertex& u_prev, ScalarVertex& u_prev_dt,
                                       std::vector<BaseEdge::Ptr>& cost_edges, std::vector<BaseEdge::Ptr>& eq_edges,
                                       std::vector<BaseEdge::Ptr>& ineq_edges);

    BaseEdge::Ptr getFinalStateCostEdge(int k, VectorVertex& xf);
    BaseEdge::Ptr getFinalStateConstraintEdge(int k, VectorVertex& xf);

    void checkAndInitializeBoundDimensions(int x_dim, int u_dim);

    // Multistage Planner Functions --> Stage Cost, FinalStageCost, StageIneq. .
    void setMultistagePlannerId(const int id);
    std::vector<int> getMultistagePlannerId() const;
    bool isMultistagePlannerSet() const;

    bool hasIntegralTerms(int k) const
    {
        if (stage_cost && stage_cost->hasIntegralTerms(k)) return true;
        if (stage_equalities && stage_equalities->hasIntegralTerms(k)) return true;
        if (stage_inequalities && stage_inequalities->hasIntegralTerms(k)) return true;
        return false;
    }

    void clear()
    {
        stage_cost.reset();
        final_stage_cost.reset();
        stage_equalities.reset();
        stage_inequalities.reset();
        final_stage_constraints.reset();
        stage_preprocessor.reset();

        x_lb = Eigen::VectorXd();
        x_ub = Eigen::VectorXd();
        u_lb = Eigen::VectorXd();
        u_ub = Eigen::VectorXd();
    }
};

}  // namespace mhp_planner

#endif  // SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_FUNCTIONS_NLP_FUNCTIONS_H_
