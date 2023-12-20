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

#ifndef SRC_OPTIMIZATION_INCLUDE_MHP_PLANNER_OPTIMIZATION_SOLVER_NLP_SOLVER_IPOPT_H_
#define SRC_OPTIMIZATION_INCLUDE_MHP_PLANNER_OPTIMIZATION_SOLVER_NLP_SOLVER_IPOPT_H_

#include <mhp_planner/solvers/nlp_solver_interface.h>
#include <memory>

#include <mhp_planner/solvers/nlp_solver_ipopt_wrapper.h>
#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>

namespace mhp_planner {

/**
 * @brief Interface to the external interior point solver IPOPT
 *
 * @ingroup optimization solver
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class SolverIpopt : public NlpSolverInterface
{
    friend class IpoptWrapper;

 public:
    using Ptr  = std::shared_ptr<SolverIpopt>;
    using UPtr = std::unique_ptr<SolverIpopt>;

    enum class LinearSolver { MUMPS, MA27, MA57, MA77, MA86, MA97, NO_SOLVER };

    // implements interface method
    NlpSolverInterface::Ptr getInstance() const override { return std::make_shared<SolverIpopt>(); }

    // implements interface method
    bool isLsqSolver() const override { return false; }

    // implements interface method
    bool initialize(OptimizationProblemInterface* problem = nullptr) override;
    // implements interface method
    SolverStatus solve(OptimizationProblemInterface& problem, bool new_structure, bool new_run, bool warm_start, double* obj_value = nullptr,
                       int* iterations = nullptr) override;

    /**@name Set solver properties */
    //@{

    bool setLinearSolver(LinearSolver solver_type);
    LinearSolver getLinearSolver() const;

    // alternative method to directly pass sovler_name to ipopt
    bool setLinearSolverByName(const std::string& solver_name);
    std::string getLinearSolverByName();

    bool setRelTolerance(double tolerance);
    double getRelTolerance() const;

    bool setDualInfTolerance(double tolerance);
    double getDualInfTolerance() const;

    bool setConstrViolTolerance(double tolerance);
    double getConstrViolTolerance() const;

    bool setComplInfTolerance(double tolerance);
    double getComplInfTolerance() const;

    bool setMuStrategy(std::string strategy);
    std::string isMuStrategy() const;

    bool setHessianApprox(std::string approx);
    std::string isHessianApprox() const;

    bool setWarmStartInitPoint(bool enabled);
    bool isWarmStartInitPoint() const;

    bool setMehrotraAlgorithm(bool enabled);
    bool isMehrotraAlgorithm() const;

    bool setPrintLevel(int print_level);  // level 0-5
    int getPrintLevel() const;

    bool setNlpScaling(std::string scaling);
    std::string isNlpScaling() const;

    void setIterations(int iterations) { _iterations = iterations; }
    int getIterations() const { return _iterations; }

    void setMaxWallTime(double max_wall_time) { _max_solve_time = max_wall_time; }
    double getMaxWallTime() const { return _max_solve_time; }

    bool setCheckDerivativesForNan(bool enabled);
    bool isCheckDerivativesForNan() const;

    // generic setter methods
    bool setIpoptOptionString(const std::string& param, const std::string& option);
    bool setIpoptOptionInt(const std::string& param, int option);
    bool setIpoptOptionNumeric(const std::string& param, double option);

    void setCacheFirstOrderDerivatives(bool active) { _cache_first_order_derivatives = active; }

    //@}

    void clear() override;

    bool fromParameterServer(const std::string& ns) override;

 protected:
    SolverStatus convertIpoptToNlpSolverStatus(Ipopt::ApplicationReturnStatus ipopt_status) const;

 private:
    Ipopt::SmartPtr<IpoptWrapper> _ipopt_nlp;
    Ipopt::SmartPtr<Ipopt::IpoptApplication> _ipopt_app;

    int _nnz_jac_constraints = 0;
    int _nnz_h_lagrangian    = 0;
    int _nnz_hes_obj         = 0;
    int _nnz_hes_eq          = 0;
    int _nnz_hes_ineq        = 0;

    Eigen::VectorXd _lambda_cache;  //! store constraint mupltipliers between subsequent IPOPT calls
    Eigen::VectorXd _zl_cache;      //! store lower bound multipliers between subsequent IPOPT calls
    Eigen::VectorXd _zu_cache;      //! store upper bound multipliers between subsequent IPOPT calls

    Eigen::VectorXd _grad_f_cache;
    Eigen::VectorXd _jac_constr_cache;

    double _max_solve_time = -1;   // use ipopt default (0: deactivate) [but should be deactivated by default]
    int _iterations        = 100;  // ipopt max iterations

    bool _cache_first_order_derivatives = false;

    LinearSolver _current_lin_solver = LinearSolver::NO_SOLVER;

    bool _initialized      = false;
    bool _allow_warm_start = false;

    int _param_msg_received = false;
};

FACTORY_REGISTER_NLP_SOLVER(SolverIpopt)

}  // namespace mhp_planner

#endif  // SRC_OPTIMIZATION_INCLUDE_MHP_PLANNER_OPTIMIZATION_SOLVER_NLP_SOLVER_IPOPT_H_
