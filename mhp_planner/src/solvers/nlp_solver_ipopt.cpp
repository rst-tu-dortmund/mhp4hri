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

#include <mhp_planner/solvers/nlp_solver_ipopt.h>

#include <mhp_planner/core/console.h>

namespace mhp_planner {

bool SolverIpopt::initialize(OptimizationProblemInterface* /*problem*/)
{
    if (_initialized) return true;

    _ipopt_nlp = new IpoptWrapper(this);
    _ipopt_app = IpoptApplicationFactory();

    if (!_param_msg_received)  // TODO(roesmann) we need a default paramerter mechanism with protobuf!
    {
        setRelTolerance(1e-3);

        setMuStrategy(std::string("adaptive"));
        setWarmStartInitPoint(true);
        setNlpScaling("gradient-based");
        setPrintLevel(2);  // Default = 5

        /*
        #ifdef __unix__
                if (!setLinearSolver(LinearSolver::MA57)) setLinearSolver(LinearSolver::MUMPS);
        #else
                setLinearSolver(LinearSolver::MUMPS);  // the default solver on systems other than linux is MUMPS
                                                       // since setLinearSolver() returns true, even if MA57 is not available
        #endif
        */
        // _ipopt_app->Options()->SetStringValue("output_file", "ipopt.out");

        // _ipopt_app->Options()->SetStringValue("hessian_approximation", "exact"); // or "limited-memory"

        // _ipopt_app->Options()->SetStringValue("derivative_test", "second-order"); // "none", "first-order", "second-order", "only-second-order"
        // _ipopt_app->Options()->SetNumericValue("derivative_test_tol", 1e-3);

        Ipopt::ApplicationReturnStatus status;
        status = _ipopt_app->Initialize();
        if (status != Ipopt::Solve_Succeeded)
        {
            PRINT_INFO("SolverIPOPT(): Error during IPOPT initialization!");
            return false;
        }
    }

    static bool copyright_printed = false;
    if (!copyright_printed)
    {
        _ipopt_app->PrintCopyrightMessage();
        copyright_printed = true;
    }

    _initialized = true;
    return true;
}

SolverStatus SolverIpopt::solve(OptimizationProblemInterface& problem, bool new_structure, bool new_run, bool warm_start, double* obj_value,
                                int* iterations)
{
    if (!_initialized)
    {
        if (!initialize(&problem)) return SolverStatus::Error;
    }

    _ipopt_nlp->setOptimizationProblem(problem);

    if (new_structure)
    {
        _nnz_jac_constraints = problem.computeCombinedSparseJacobiansNNZ(false, true, true);

        problem.computeSparseHessiansNNZ(_nnz_hes_obj, _nnz_hes_eq, _nnz_hes_ineq, true);
        _nnz_h_lagrangian = _nnz_hes_obj + _nnz_hes_eq + _nnz_hes_ineq;

        _lambda_cache.resize(problem.getEqualityDimension() + problem.getInequalityDimension());
        _lambda_cache.setZero();

        _zl_cache.resize(problem.getParameterDimension());
        _zl_cache.setZero();

        _zu_cache.resize(problem.getParameterDimension());
        _zu_cache.setZero();
    }

    // set max number of iterations
    _ipopt_app->Options()->SetIntegerValue("max_iter", _iterations);

    // if internal warm start is allowed, do so if requested
    setWarmStartInitPoint(_allow_warm_start && warm_start);

    if (_max_solve_time > 0)
        _ipopt_app->Options()->SetNumericValue("max_wall_time", _max_solve_time);
    else if (_max_solve_time <= 0)
    {
        ROS_WARN("SolverIpopt::solve(): max_solve_time is not set. Using default value of 10e6 seconds.");
        _ipopt_app->Options()->SetNumericValue("max_wall_time", 10e6);
    }

    Ipopt::ApplicationReturnStatus ipopt_status;
    if (new_structure)
        ipopt_status = _ipopt_app->OptimizeTNLP(_ipopt_nlp);
    else
        ipopt_status = _ipopt_app->ReOptimizeTNLP(_ipopt_nlp);

    // store some statistics
    if (obj_value) *obj_value = _ipopt_app->Statistics()->FinalObjective();
    if (iterations) *iterations = _ipopt_app->Statistics()->IterationCount();

    return convertIpoptToNlpSolverStatus(ipopt_status);
}

bool SolverIpopt::setLinearSolver(LinearSolver solver_type)
{
    if (!Ipopt::IsValid(_ipopt_app)) return false;

    bool success = false;

    switch (solver_type)
    {
        case LinearSolver::MUMPS:
            success = _ipopt_app->Options()->SetStringValue("linear_solver", "mumps");
            break;
        case LinearSolver::MA27:
            success = _ipopt_app->Options()->SetStringValue("linear_solver", "ma27");
            break;
        case LinearSolver::MA57:
            success = _ipopt_app->Options()->SetStringValue("linear_solver", "ma57");
            break;
        case LinearSolver::MA77:
            success = _ipopt_app->Options()->SetStringValue("linear_solver", "ma77");
            break;
        case LinearSolver::MA86:
            success = _ipopt_app->Options()->SetStringValue("linear_solver", "ma86");
            break;
        case LinearSolver::MA97:
            success = _ipopt_app->Options()->SetStringValue("linear_solver", "ma97");
            break;
        default:
            success = false;
    }

    if (success)
    {
        _current_lin_solver = solver_type;
        return true;
    }
    return false;
}

SolverIpopt::LinearSolver SolverIpopt::getLinearSolver() const
{
    if (!Ipopt::IsValid(_ipopt_app)) return LinearSolver::NO_SOLVER;

    std::string linear_solver;
    if (!_ipopt_app->Options()->GetStringValue("linear_solver", linear_solver, "")) return LinearSolver::NO_SOLVER;

    if (!linear_solver.compare("mumps")) return LinearSolver::MUMPS;
    if (!linear_solver.compare("ma27")) return LinearSolver::MA27;
    if (!linear_solver.compare("ma57")) return LinearSolver::MA57;
    if (!linear_solver.compare("ma77")) return LinearSolver::MA77;
    if (!linear_solver.compare("ma86")) return LinearSolver::MA86;
    if (!linear_solver.compare("ma97")) return LinearSolver::MA97;

    return LinearSolver::NO_SOLVER;
}

bool SolverIpopt::setLinearSolverByName(const std::string& solver_name)
{
    if (!Ipopt::IsValid(_ipopt_app)) return false;
    return _ipopt_app->Options()->SetStringValue("linear_solver", solver_name);
}

std::string SolverIpopt::getLinearSolverByName()
{
    if (!Ipopt::IsValid(_ipopt_app)) return "";
    std::string linear_solver;
    if (!_ipopt_app->Options()->GetStringValue("linear_solver", linear_solver, "")) return "";
    return linear_solver;
}

bool SolverIpopt::setRelTolerance(double tolerance) { return _ipopt_app->Options()->SetNumericValue("tol", tolerance); }
double SolverIpopt::getRelTolerance() const
{
    double val = MHP_PLANNER_INF_DBL;
    _ipopt_app->Options()->GetNumericValue("tol", val, "");
    return val;
}

bool SolverIpopt::setDualInfTolerance(double tolerance) { return _ipopt_app->Options()->SetNumericValue("dual_inf_tol", tolerance); }
double SolverIpopt::getDualInfTolerance() const
{
    double val = MHP_PLANNER_INF_DBL;
    _ipopt_app->Options()->GetNumericValue("dual_inf_tol", val, "");
    return val;
}

bool SolverIpopt::setConstrViolTolerance(double tolerance) { return _ipopt_app->Options()->SetNumericValue("constr_viol_tol", tolerance); }
double SolverIpopt::getConstrViolTolerance() const
{
    double val = MHP_PLANNER_INF_DBL;
    _ipopt_app->Options()->GetNumericValue("constr_viol_tol", val, "");
    return val;
}

bool SolverIpopt::setComplInfTolerance(double tolerance) { return _ipopt_app->Options()->SetNumericValue("compl_inf_tol", tolerance); }
double SolverIpopt::getComplInfTolerance() const
{
    double val = MHP_PLANNER_INF_DBL;
    _ipopt_app->Options()->GetNumericValue("compl_inf_tol", val, "");
    return val;
}

bool SolverIpopt::setMuStrategy(std::string strategy) { return _ipopt_app->Options()->SetStringValue("mu_strategy", strategy); }
std::string SolverIpopt::isMuStrategy() const
{
    std::string opt;
    _ipopt_app->Options()->GetStringValue("mu_strategy", opt, "");
    return opt;
}

bool SolverIpopt::setHessianApprox(std::string approx) { return _ipopt_app->Options()->SetStringValue("hessian_approximation", approx); }
std::string SolverIpopt::isHessianApprox() const
{
    std::string opt;
    _ipopt_app->Options()->GetStringValue("hessian_approximation", opt, "");
    return opt;
}

bool SolverIpopt::setWarmStartInitPoint(bool enabled)
{
    if (enabled) return _ipopt_app->Options()->SetStringValue("warm_start_init_point", "yes");
    return _ipopt_app->Options()->SetStringValue("warm_start_init_point", "no");
}
bool SolverIpopt::isWarmStartInitPoint() const
{
    std::string opt;
    _ipopt_app->Options()->GetStringValue("warm_start_init_point", opt, "");
    return opt.compare("yes") == 0 ? true : false;
}

bool SolverIpopt::setMehrotraAlgorithm(bool enabled)
{
    if (enabled) return _ipopt_app->Options()->SetStringValue("mehrotra_algorithm", "yes");
    return _ipopt_app->Options()->SetStringValue("mehrotra_algorithm", "no");
}
bool SolverIpopt::isMehrotraAlgorithm() const
{
    std::string opt;
    _ipopt_app->Options()->GetStringValue("mehrotra_algorithm", opt, "");
    return opt.compare("yes") == 0 ? true : false;
}

bool SolverIpopt::setPrintLevel(int print_level) { return _ipopt_app->Options()->SetIntegerValue("print_level", print_level); }  // level 0-5
int SolverIpopt::getPrintLevel() const
{
    int val = -1;
    _ipopt_app->Options()->GetIntegerValue("print_level", val, "");
    return val;
}

bool SolverIpopt::setNlpScaling(std::string scaling) { return _ipopt_app->Options()->SetStringValue("nlp_scaling_method", scaling); }
std::string SolverIpopt::isNlpScaling() const
{
    std::string opt;
    _ipopt_app->Options()->GetStringValue("nlp_scaling_method", opt, "");
    return opt;
}

bool SolverIpopt::setCheckDerivativesForNan(bool enabled)
{
    if (enabled) return _ipopt_app->Options()->SetStringValue("check_derivatives_for_naninf", "yes");
    return _ipopt_app->Options()->SetStringValue("check_derivatives_for_naninf", "no");
}
bool SolverIpopt::isCheckDerivativesForNan() const
{
    std::string opt;
    _ipopt_app->Options()->GetStringValue("check_derivatives_for_naninf", opt, "");
    return opt.compare("yes") == 0 ? true : false;
}

bool SolverIpopt::setIpoptOptionString(const std::string& param, const std::string& option)
{
    if (!Ipopt::IsValid(_ipopt_app)) return false;
    return _ipopt_app->Options()->SetStringValue(param, option);
}

bool SolverIpopt::setIpoptOptionInt(const std::string& param, int option)
{
    if (!Ipopt::IsValid(_ipopt_app)) return false;
    return _ipopt_app->Options()->SetIntegerValue(param, option);
}

bool SolverIpopt::setIpoptOptionNumeric(const std::string& param, double option)
{
    if (!Ipopt::IsValid(_ipopt_app)) return false;
    return _ipopt_app->Options()->SetNumericValue(param, option);
}

SolverStatus SolverIpopt::convertIpoptToNlpSolverStatus(Ipopt::ApplicationReturnStatus ipopt_status) const
{
    switch (ipopt_status)
    {
        case Ipopt::Solve_Succeeded:
        case Ipopt::Solved_To_Acceptable_Level: {
            return SolverStatus::Converged;
        }
        case Ipopt::Search_Direction_Becomes_Too_Small:
        case Ipopt::User_Requested_Stop:
        case Ipopt::Feasible_Point_Found:
        case Ipopt::Maximum_CpuTime_Exceeded:
        case Ipopt::Maximum_WallTime_Exceeded:
        case Ipopt::Maximum_Iterations_Exceeded: {
            return SolverStatus::EarlyTerminated;
        }
        case Ipopt::Infeasible_Problem_Detected:
        case Ipopt::Diverging_Iterates:
        case Ipopt::Restoration_Failed:
        case Ipopt::Not_Enough_Degrees_Of_Freedom:
        case Ipopt::Invalid_Problem_Definition: {
            return SolverStatus::Infeasible;
        }
        default: {
        }
    };
    return SolverStatus::Error;
}

void SolverIpopt::clear()
{
    _nnz_jac_constraints = 0;
    _nnz_h_lagrangian    = 0;
    _nnz_hes_obj         = 0;
    _nnz_hes_eq          = 0;
    _nnz_hes_ineq        = 0;

    // the initialize() function does not fill any problem dependent caches, so let's keep it initialized...
    // _initialized = false;
    // _ipopt_nlp = nullptr;
    // _ipopt_app = nullptr;
}

bool SolverIpopt::fromParameterServer(const std::string& ns)
{
    if (!_initialized) initialize();  // we need valid objects // TODO(roesmann): this is not nice and not consistent in the project

    ros::NodeHandle nh;
    setIterations(nh.param<int>(ns + "/iterations", 100));
    setMaxWallTime(nh.param<double>(ns + "/max_wall_time", 0.0));

    std::string linear_solver_type;
    nh.getParam(ns + "/linear_solver", linear_solver_type);
    if (linear_solver_type == "MUMPS")
    {
        if (!setLinearSolver(LinearSolver::MUMPS))
        {
            PRINT_ERROR("LinearSolver::MUMPS: not supported with current IPOPT installation.");
            return false;
        }
    }
    else if (linear_solver_type == "MA27")
    {
        if (!setLinearSolver(LinearSolver::MA27))
        {
            PRINT_ERROR("LinearSolver::MA27: not supported with current IPOPT installation.");
            return false;
        }
    }
    else if (linear_solver_type == "MA57")
    {
        if (!setLinearSolver(LinearSolver::MA57))
        {
            PRINT_ERROR("LinearSolver::MA57: not supported with current IPOPT installation.");
            return false;
        }
    }
    else if (linear_solver_type == "MA77")
    {
        if (!setLinearSolver(LinearSolver::MA77))
        {
            PRINT_ERROR("LinearSolver::MA77: not supported with current IPOPT installation.");
            return false;
        }
    }
    else if (linear_solver_type == "MA86")
    {
        if (!setLinearSolver(LinearSolver::MA86))
        {
            PRINT_ERROR("LinearSolver::MA86: not supported with current IPOPT installation.");
            return false;
        }
    }
    else if (linear_solver_type == "MA97")
    {
        if (!setLinearSolver(LinearSolver::MA97))
        {
            PRINT_ERROR("LinearSolver::MA97: not supported with current IPOPT installation.");
            return false;
        }
    }
    else
    {
        PRINT_ERROR("SolverIpopt::fromParameterServer(): selected linear solver is currently not supported.");
        return false;
    }

    bool success = true;
    if (!setRelTolerance(nh.param<double>(ns + "/rel_tolerance", 0.5)))
    {
        PRINT_ERROR("setRelTolerance() failed.");
        success = false;
    };

    if (!setMuStrategy(nh.param<std::string>(ns + "/mu_strategy", "adaptive")))
    {
        PRINT_ERROR("setMuStrategyAdaptive() failed");
        success = false;
    };

    if (!setHessianApprox(nh.param<std::string>(ns + "/hessian_approximation", "limited-memory")))
    {
        PRINT_ERROR("setHessianApprox() failed");
        success = false;
    };

    bool mehrotra;
    nh.getParam(ns + "/mehrotra_algorithm", mehrotra);
    if (!setMehrotraAlgorithm(mehrotra))
    {
        PRINT_ERROR("setMehrotraAlgorithm() failed");
        success = false;
    };

    if (!setNlpScaling(nh.param<std::string>(ns + "/nlp_scaling_method", "gradient-based")))
    {
        PRINT_ERROR("setNlpScaling() failed");
        success = false;
    };

    if (!setPrintLevel(nh.param<int>(ns + "/print_level", 2)))
    {
        PRINT_ERROR("setPrintLevel() failed");
        success = false;
    };

    bool derivativetest;
    nh.getParam(ns + "/check_derivatives_for_naninf", derivativetest);
    if (!setCheckDerivativesForNan(derivativetest))
    {
        PRINT_ERROR("setCheckDerivativesForNan() failed");
        success = false;
    };

    nh.getParam(ns + "/warm_start_init_point", _allow_warm_start);
    if (!setWarmStartInitPoint(_allow_warm_start))
    {
        PRINT_ERROR("SolverIpopt::fromParameterServer(): Could not read parameter warm_start_init_point");
        success = false;
    };

    _cache_first_order_derivatives = true;

    Ipopt::ApplicationReturnStatus status;
    status = _ipopt_app->Initialize();
    if (status != Ipopt::Solve_Succeeded)
    {
        if (!success)
            PRINT_ERROR(
                "SolverIpopt::fromParameterServer(): Error during IPOPT initialization! Maybe some parameters are not supported with current "
                "installation");
        PRINT_WARNING(
            "SolverIpopt::fromParameterServer(): Error during IPOPT initialization! Maybe some parameters are not supported with current "
            "installation");
    }

    _param_msg_received = true;

    return success;
}

}  // namespace mhp_planner
