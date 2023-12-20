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

#ifndef SRC_OPTIMIZATION_INCLUDE_MHP_PLANNER_OPTIMIZATION_SOLVER_NLP_SOLVER_INTERFACE_H_
#define SRC_OPTIMIZATION_INCLUDE_MHP_PLANNER_OPTIMIZATION_SOLVER_NLP_SOLVER_INTERFACE_H_

#include <mhp_planner/core/factory.h>
#include <mhp_planner/core/types.h>
#include <mhp_planner/hypergraph/optimization_problem_interface.h>
#include <mhp_planner/hypergraph/types.h>
#include <ros/ros.h>
#include <memory>

namespace mhp_planner {

/**
 * @brief Generic interface for solver implementations
 *
 * @ingroup optimization solver
 *
 * This class can be used to generically define solver back-ends
 * for optimization problems with objectives, equality constraints,
 * inequality constraints, box constraints and their current parameter state:
 *
 * \f[
 *    \min f(x), \ f : \mathbb{R}^n \to \mathbb{R}^m \\
 *    \text{subject to:} \\
 *    ceq(x) = 0 \\
 *    c(x) < 0 \\
 *    lb < x < ub
 * \f]
 *
 * The optimization problem is usually defined in terms of
 * an OptimizationProblemInterface instance and the solver
 * needs to check whether 1st and/or 2nd order derivatives
 * in dense or sparse form are required.
 *
 * @see OptimizationProblemInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class NlpSolverInterface
{
 public:
    using Ptr  = std::shared_ptr<NlpSolverInterface>;
    using UPtr = std::unique_ptr<NlpSolverInterface>;

    //! Virtual destructor
    virtual ~NlpSolverInterface() {}

    //! Return a newly created instance of the current solver
    virtual Ptr getInstance() const = 0;

    //! Return true if the solver onyl supports costs in lsq form
    virtual bool isLsqSolver() const = 0;

    virtual bool initialize(OptimizationProblemInterface* problem = nullptr) { return true; }

    virtual SolverStatus solve(OptimizationProblemInterface& problem, bool new_structure, bool new_run, bool warm_start, double* obj_value = nullptr,
                               int* iterations = nullptr) = 0;

    //! Clear internal caches
    virtual void clear() = 0;

    virtual bool fromParameterServer(const std::string& ns) = 0;
};

using NlpSolverFactory = Factory<NlpSolverInterface>;
#define FACTORY_REGISTER_NLP_SOLVER(type) FACTORY_REGISTER_OBJECT(type, NlpSolverInterface)

}  // namespace mhp_planner

#endif  // SRC_OPTIMIZATION_INCLUDE_MHP_PLANNER_OPTIMIZATION_SOLVER_NLP_SOLVER_INTERFACE_H_
