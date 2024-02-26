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

#ifndef SRC_OPTIMIZATION_INCLUDE_MHP_PLANNER_OPTIMIZATION_OPTIMIZATION_PROBLEM_INTERFACE_H_
#define SRC_OPTIMIZATION_INCLUDE_MHP_PLANNER_OPTIMIZATION_OPTIMIZATION_PROBLEM_INTERFACE_H_

#include <mhp_planner/core/macros.h>
#include <mhp_planner/core/types.h>

#include <Eigen/Sparse>

#include <functional>
#include <memory>

namespace mhp_planner {

/**
 * @brief Generic interface for optimization problem definitions
 *
 * @ingroup optimization
 *
 * This class can be used to generically define optimization problems
 * with objectives, equality constraints, inequality constraints,
 * box constraints and their current parameter state.
 *
 * \f[
 *    \min f(x), \ f : \mathbb{R}^n \to \mathbb{R}^m \\
 *    \text{subject to:} \\
 *    ceq(x) = 0 \\
 *    c(x) < 0 \\
 *    lb < x < ub
 * \f]
 *
 * First order and second order derivatives can be provided but
 * their actual usage depends on the SolverInterface back-end
 * which solves the optimization problem. Some solvers may require
 * the implementation of those methods.
 *
 * Solvers determine a parameter increment in each of their iterations
 * so this class provides a method applyIncrement() to modify the underlying
 * optimization parameter set. In order to convinentially store and restore
 * the current parameter set from the solver backend side,
 * appropriate backup methods must be provided by subclasses.
 *
 * @see SolverInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class OptimizationProblemInterface
{
 public:
    using Ptr  = std::shared_ptr<OptimizationProblemInterface>;
    using UPtr = std::unique_ptr<OptimizationProblemInterface>;

    virtual ~OptimizationProblemInterface() {}

    virtual void clear() {}

    /** @name Specify the dimension of the optimization problem  */
    //@{

    //! Get dimension of the objective (should be zero or one)
    virtual int getNonLsqObjectiveDimension() = 0;
    //! Total dimension of least-squares objective function terms
    virtual int getLsqObjectiveDimension() = 0;
    //! Get dimension of the objective (should be zero or one, includes Lsq objectives if present)
    virtual int getObjectiveDimension() = 0;

    //! Total dimension of equality constraints
    virtual int getEqualityDimension() = 0;
    //! Total dimension of general inequality constraints
    virtual int getInequalityDimension() = 0;
    //! Effictive dimension of the optimization parameter set (changeable, non-fixed part)
    virtual int getParameterDimension() = 0;

    //@}

    /** @name Generic interface to interact with the underlying parameter/variables representation  */
    //@{

    virtual void applyIncrement(const Eigen::Ref<const Eigen::VectorXd>& increment);

    virtual void applyIncrement(int idx, double increment);

    virtual double getParameterValue(int idx) = 0;

    virtual void setParameterValue(int idx, double x) = 0;

    virtual void getParameterVector(Eigen::Ref<Eigen::VectorXd> x);

    virtual void setParameterVector(const Eigen::Ref<const Eigen::VectorXd>& x);

    //!< Push the current parameter set to some backup stack
    virtual void backupParameters() = 0;
    //!< Restore parameter set from the last backup and keep backup if desired
    virtual void restoreBackupParameters(bool keep_backup) = 0;
    //!< Discard last backup (or all)
    virtual void discardBackupParameters(bool all = false) = 0;

    //@}

    /** @name Specify properties of the optimization problem  */
    //@{

    virtual bool isLeastSquaresProblem() const = 0;

    //@}

    /** @name Access bounds  */
    //@{

    virtual double getLowerBound(int idx) = 0;

    virtual double getUpperBound(int idx) = 0;

    virtual void setLowerBound(int idx, double lb) = 0;

    virtual void setUpperBound(int idx, double ub) = 0;

    virtual void setBounds(const Eigen::Ref<const Eigen::VectorXd>& lb, const Eigen::Ref<const Eigen::VectorXd>& ub);

    virtual void getBounds(Eigen::Ref<Eigen::VectorXd> lb, Eigen::Ref<Eigen::VectorXd> ub);

    //@}

    /** @name Specify main equations of the optimization problem  */
    //@{

    virtual void computeValuesLsqObjective(Eigen::Ref<Eigen::VectorXd> values) = 0;

    virtual double computeValueNonLsqObjective() = 0;

    virtual double computeValueObjective();

    virtual void computeValuesEquality(Eigen::Ref<Eigen::VectorXd> values) = 0;

    virtual void computeValuesInequality(Eigen::Ref<Eigen::VectorXd> values) = 0;

    virtual void computeValues(double& non_lsq_obj_value, Eigen::Ref<Eigen::VectorXd> lsq_obj_values, Eigen::Ref<Eigen::VectorXd> eq_values,
                               Eigen::Ref<Eigen::VectorXd> ineq_values)
    {
        non_lsq_obj_value = computeValueNonLsqObjective();
        computeValuesLsqObjective(lsq_obj_values);
        computeValuesEquality(eq_values);
        computeValuesInequality(ineq_values);
    }

    //@}

    /** @name Methods for dealing with bounds  */
    //@{

    virtual int finiteCombinedBoundsDimension();

    virtual int finiteBoundsDimension();

    //@}

    virtual void computeValuesActiveInequality(Eigen::Ref<Eigen::VectorXd> values, double weight = 1.0);

    virtual void computeDistanceFiniteCombinedBounds(Eigen::Ref<Eigen::VectorXd> values);

    virtual void computeLowerAndUpperBoundDiff(Eigen::Ref<Eigen::VectorXd> lb_minus_x, Eigen::Ref<Eigen::VectorXd> ub_minus_x);

    virtual void getParametersAndBoundsFinite(Eigen::Ref<Eigen::VectorXd> lb_finite_bounds, Eigen::Ref<Eigen::VectorXd> ub_finite_bounds,
                                              Eigen::Ref<Eigen::VectorXd> x_finite_bounds);

    virtual void computeGradientObjective(Eigen::Ref<Eigen::VectorXd> gradient);

    virtual void computeGradientNonLsqObjective(Eigen::Ref<Eigen::VectorXd> gradient);

    virtual void computeDenseJacobianLsqObjective(Eigen::Ref<Eigen::MatrixXd> jacobian, const double* multipliers = nullptr);
    virtual int computeSparseJacobianLsqObjectiveNNZ();
    virtual void computeSparseJacobianLsqObjectiveStructure(Eigen::Ref<Eigen::VectorXi> i_row, Eigen::Ref<Eigen::VectorXi> j_col);
    virtual void computeSparseJacobianLsqObjectiveValues(Eigen::Ref<Eigen::VectorXd> values, const double* multipliers = nullptr);
    virtual void computeSparseJacobianLsqObjective(Eigen::SparseMatrix<double>& jacobian, const double* multipliers = nullptr);

    virtual void computeDenseJacobianEqualities(Eigen::Ref<Eigen::MatrixXd> jacobian, const double* multipliers = nullptr);
    virtual int computeSparseJacobianEqualitiesNNZ();
    virtual void computeSparseJacobianEqualitiesStructure(Eigen::Ref<Eigen::VectorXi> i_row, Eigen::Ref<Eigen::VectorXi> j_col);
    virtual void computeSparseJacobianEqualitiesValues(Eigen::Ref<Eigen::VectorXd> values, const double* multipliers = nullptr);
    virtual void computeSparseJacobianEqualities(Eigen::SparseMatrix<double>& jacobian, const double* multipliers = nullptr);

    virtual void computeDenseJacobianInequalities(Eigen::Ref<Eigen::MatrixXd> jacobian, const double* multipliers = nullptr);
    virtual int computeSparseJacobianInequalitiesNNZ();
    virtual void computeSparseJacobianInequalitiesStructure(Eigen::Ref<Eigen::VectorXi> i_row, Eigen::Ref<Eigen::VectorXi> j_col);
    virtual void computeSparseJacobianInequalitiesValues(Eigen::Ref<Eigen::VectorXd> values, const double* multipliers = nullptr);
    virtual void computeSparseJacobianInequalities(Eigen::SparseMatrix<double>& jacobian, const double* multipliers = nullptr);

    virtual void computeDenseJacobianActiveInequalities(Eigen::Ref<Eigen::MatrixXd> jacobian, double weight = 1.0);
    virtual void computeSparseJacobianActiveInequalitiesValues(Eigen::Ref<Eigen::VectorXd> values, double weight = 1.0);
    virtual void computeSparseJacobianActiveInequalities(Eigen::SparseMatrix<double>& jacobian, double weight = 1.0);

    virtual void computeDenseJacobianFiniteCombinedBounds(Eigen::Ref<Eigen::MatrixXd> jacobian, double weight = 1.0);
    virtual int computeSparseJacobianFiniteCombinedBoundsNNZ();
    virtual void computeSparseJacobianFiniteCombinedBoundsStructure(Eigen::Ref<Eigen::VectorXi> i_row, Eigen::Ref<Eigen::VectorXi> j_col);
    virtual void computeSparseJacobianFiniteCombinedBoundsValues(Eigen::Ref<Eigen::VectorXd> values, double weight = 1.0);
    virtual void computeSparseJacobianFiniteCombinedBounds(Eigen::SparseMatrix<double>& jacobian, double weight = 1.0);

    virtual void computeDenseJacobianFiniteCombinedBoundsIdentity(Eigen::Ref<Eigen::MatrixXd> jacobian);

    virtual void computeDenseJacobians(Eigen::Ref<Eigen::VectorXd> gradient_non_lsq_obj, Eigen::Ref<Eigen::MatrixXd> jacobian_lsq_obj,
                                       Eigen::Ref<Eigen::MatrixXd> jacobian_eq, Eigen::Ref<Eigen::MatrixXd> jacobian_ineq,
                                       const double* multipliers_lsq_obj = nullptr, const double* multipliers_eq = nullptr,
                                       const double* multipliers_ineq = nullptr, bool active_ineq = false, double active_ineq_weight = 1.0);
    virtual void computeSparseJacobiansNNZ(int& nnz_lsq_obj, int& nnz_eq, int& nnz_ineq);
    virtual void computeSparseJacobiansStructure(Eigen::Ref<Eigen::VectorXi> i_row_obj, Eigen::Ref<Eigen::VectorXi> j_col_obj,
                                                 Eigen::Ref<Eigen::VectorXi> i_row_eq, Eigen::Ref<Eigen::VectorXi> j_col_eq,
                                                 Eigen::Ref<Eigen::VectorXi> i_row_ineq, Eigen::Ref<Eigen::VectorXi> j_col_ineq);
    virtual void computeSparseJacobiansValues(Eigen::Ref<Eigen::VectorXd> values_obj, Eigen::Ref<Eigen::VectorXd> values_eq,
                                              Eigen::Ref<Eigen::VectorXd> values_ineq, const double* multipliers_obj = nullptr,
                                              const double* multipliers_eq = nullptr, const double* multipliers_ineq = nullptr,
                                              bool active_ineq = false, double active_ineq_weight = 1.0);
    virtual void computeSparseJacobians(Eigen::SparseMatrix<double>& jacobian_lsq_obj, Eigen::SparseMatrix<double>& jacobian_eq,
                                        Eigen::SparseMatrix<double>& jacobian_ineq, const double* multipliers_lsq_obj = nullptr,
                                        const double* multipliers_eq = nullptr, const double* multipliers_ineq = nullptr, bool active_ineq = false,
                                        double active_ineq_weight = 1.0);

    virtual void computeCombinedSparseJacobian(Eigen::SparseMatrix<double>& jacobian, bool objective_lsq, bool equality, bool inequality,
                                               bool finite_combined_bounds, bool active_ineq = false, double weight_eq = 1.0,
                                               double weight_ineq = 1.0, double weight_bounds = 1.0);

    virtual int computeCombinedSparseJacobiansNNZ(bool objective_lsq = true, bool equality = true, bool inequality = true);
    virtual void computeCombinedSparseJacobiansStructure(Eigen::Ref<Eigen::VectorXi> i_row, Eigen::Ref<Eigen::VectorXi> j_col,
                                                         bool objective_lsq = true, bool equality = true, bool inequality = true);
    virtual void computeCombinedSparseJacobiansValues(Eigen::Ref<Eigen::VectorXd> values, bool objective_lsq = true, bool equality = true,
                                                      bool inequality = true, const double* multipliers_obj = nullptr,
                                                      const double* multipliers_eq = nullptr, const double* multipliers_ineq = nullptr);

    // useful for IPOPT (w/ hessian-approx)
    virtual void computeGradientObjectiveAndCombinedSparseJacobiansValues(Eigen::Ref<Eigen::VectorXd> gradient,
                                                                          Eigen::Ref<Eigen::VectorXd> jac_values, bool equality = true,
                                                                          bool inequality = true, const double* multipliers_eq = nullptr,
                                                                          const double* multipliers_ineq = nullptr);

    virtual void computeDenseHessianObjective(const Eigen::Ref<const Eigen::MatrixXd>& jacobian, Eigen::Ref<Eigen::MatrixXd> hessian,
                                              const double* multipliers = nullptr, bool jacob_scaled = true);

    virtual void computeDenseHessianObjective(Eigen::Ref<Eigen::MatrixXd> hessian, double multiplier = 1.0);
    virtual int computeSparseHessianObjectiveNNZ(bool lower_part_only = false);
    virtual void computeSparseHessianObjectiveStructure(Eigen::Ref<Eigen::VectorXi> i_row, Eigen::Ref<Eigen::VectorXi> j_col,
                                                        bool lower_part_only = false);
    virtual void computeSparseHessianObjectiveValues(Eigen::Ref<Eigen::VectorXd> values, double multiplier = 1.0, bool lower_part_only = false);
    virtual void computeSparseHessianObjective(Eigen::SparseMatrix<double>& hessian, double multiplier = 1.0);

    virtual void computeSparseHessianObjectiveLL(Eigen::SparseMatrix<double, Eigen::ColMajor, long long>& H, const Eigen::VectorXi* col_nnz = nullptr,
                                                 bool upper_part_only = false);
    virtual void computeSparseHessianObjectiveNNZperCol(Eigen::Ref<Eigen::VectorXi> col_nnz, bool upper_part_only = false);

    virtual void computeDenseHessianEqualities(Eigen::Ref<Eigen::MatrixXd> hessian, const double* multipliers = nullptr);
    virtual int computeSparseHessianEqualitiesNNZ(bool lower_part_only = false);
    virtual void computeSparseHessianEqualitiesStructure(Eigen::Ref<Eigen::VectorXi> i_row, Eigen::Ref<Eigen::VectorXi> j_col,
                                                         bool lower_part_only = false);
    virtual void computeSparseHessianEqualitiesValues(Eigen::Ref<Eigen::VectorXd> values, const double* multipliers = nullptr,
                                                      bool lower_part_only = false);
    virtual void computeSparseHessianEqualities(Eigen::SparseMatrix<double>& hessian, const double* multipliers = nullptr);

    virtual void computeDenseHessianInequalities(Eigen::Ref<Eigen::MatrixXd> hessian, const double* multipliers = nullptr);
    virtual int computeSparseHessianInequalitiesNNZ(bool lower_part_only = false);
    virtual void computeSparseHessianInequalitiesStructure(Eigen::Ref<Eigen::VectorXi> i_row, Eigen::Ref<Eigen::VectorXi> j_col,
                                                           bool lower_part_only = false);
    virtual void computeSparseHessianInequalitiesValues(Eigen::Ref<Eigen::VectorXd> values, const double* multipliers = nullptr,
                                                        bool lower_part_only = false);
    virtual void computeSparseHessianInequalities(Eigen::SparseMatrix<double>& hessian, const double* multipliers = nullptr);

    //    virtual void computeDenseJacobianHessianObjective(Eigen::Ref<Eigen::MatrixXd> jacobian, Eigen::Ref<Eigen::MatrixXd> hessian,
    //                                                      const double* multipliers = nullptr);
    //    virtual void computeDenseJacobianHessianEqualities(Eigen::Ref<Eigen::MatrixXd> jacobian, Eigen::Ref<Eigen::MatrixXd> hessian,
    //                                                       const double* multipliers = nullptr);
    //    virtual void computeDenseJacobianHessianInequalities(Eigen::Ref<Eigen::MatrixXd> jacobian, Eigen::Ref<Eigen::MatrixXd> hessian,
    //                                                         const double* multipliers = nullptr);

    virtual void computeDenseHessians(Eigen::Ref<Eigen::MatrixXd> hessian_obj, Eigen::Ref<Eigen::MatrixXd> hessian_eq,
                                      Eigen::Ref<Eigen::MatrixXd> hessian_ineq, double multiplier_obj = 1.0, const double* multipliers_eq = nullptr,
                                      const double* multipliers_ineq = nullptr);
    virtual void computeSparseHessians(Eigen::SparseMatrix<double>& hessian_obj, Eigen::SparseMatrix<double>& hessian_eq,
                                       Eigen::SparseMatrix<double>& hessian_ineq, double multiplier_obj = 1.0, const double* multipliers_eq = nullptr,
                                       const double* multipliers_ineq = nullptr);
    virtual void computeSparseHessiansNNZ(int& nnz_obj, int& nnz_eq, int& nnz_ineq, bool lower_part_only = false);
    virtual void computeSparseHessiansStructure(Eigen::Ref<Eigen::VectorXi> i_row_obj, Eigen::Ref<Eigen::VectorXi> j_col_obj,
                                                Eigen::Ref<Eigen::VectorXi> i_row_eq, Eigen::Ref<Eigen::VectorXi> j_col_eq,
                                                Eigen::Ref<Eigen::VectorXi> i_row_ineq, Eigen::Ref<Eigen::VectorXi> j_col_ineq,
                                                bool lower_part_only = false);
    virtual void computeSparseHessiansValues(Eigen::Ref<Eigen::VectorXd> values_obj, Eigen::Ref<Eigen::VectorXd> values_eq,
                                             Eigen::Ref<Eigen::VectorXd> values_ineq, double multiplier_obj = 1.0,
                                             const double* multipliers_eq = nullptr, const double* multipliers_ineq = nullptr,
                                             bool lower_part_only = false);

    virtual void computeSparseHessianLagrangian(Eigen::SparseMatrix<double, Eigen::ColMajor, long long>& H, const double* multipliers_eq,
                                                const double* multipliers_ineq, const Eigen::VectorXi* col_nnz = nullptr,
                                                bool upper_part_only = false);
    virtual void computeSparseHessianLagrangianNNZperCol(Eigen::Ref<Eigen::VectorXi> col_nnz,
                                                         bool upper_part_only);  // TODO(roesmann): lower_part_only? does it make sense for NNZ?

    /** @name Methods specialized for QP forms  */
    //@{

    virtual void computeSparseJacobianTwoSideBoundedLinearForm(Eigen::SparseMatrix<double, Eigen::ColMajor, long long>& A, bool include_finite_bounds,
                                                               const Eigen::VectorXi* col_nnz = nullptr);
    virtual void computeSparseJacobianTwoSideBoundedLinearFormNNZPerColumn(Eigen::Ref<Eigen::VectorXi> col_nnz, bool include_finite_bounds);

    virtual int computeSparseJacobianTwoSideBoundedLinearFormNNZ(bool include_finite_bounds);
    virtual void computeSparseJacobianTwoSideBoundedLinearFormStructure(Eigen::Ref<Eigen::VectorXi> i_row, Eigen::Ref<Eigen::VectorXi> j_col,
                                                                        bool include_finite_bounds);
    virtual void computeSparseJacobianTwoSideBoundedLinearFormValues(Eigen::Ref<Eigen::VectorXd> values, bool include_finite_bounds);

    virtual void computeSparseJacobianTwoSideBoundedLinearFormAndHessianLagrangian(
        Eigen::SparseMatrix<double, Eigen::ColMajor, long long>& H, const double* multipliers_eq, const double* multipliers_ineq,
        Eigen::SparseMatrix<double, Eigen::ColMajor, long long>& A, bool include_finite_bounds, const Eigen::VectorXi* col_nnz_H = nullptr,
        const Eigen::VectorXi* col_nnz_A = nullptr, bool upper_part_only_H = false);

    virtual void computeBoundsForTwoSideBoundedLinearForm(Eigen::Ref<Eigen::VectorXd> lbA, Eigen::Ref<Eigen::VectorXd> ubA,
                                                          bool include_finite_bounds);

    //@}

    virtual void computeSparseJacobianTwoSideBoundedLinearFormAndHessianObjective(Eigen::SparseMatrix<double, Eigen::ColMajor, long long>& H,
                                                                                  Eigen::SparseMatrix<double, Eigen::ColMajor, long long>& A,
                                                                                  bool include_finite_bounds, const Eigen::VectorXi* col_nnz_H,
                                                                                  const Eigen::VectorXi* col_nnz_A, bool upper_part_only_H);

    //! Check if a function taking the parameter value and unfixed-idx is true for all unfixed parameter values.
    virtual bool checkIfAllUnfixedParam(std::function<bool(double, int)> fun);

    virtual void fromParameterServer(const std::string& ns) = 0;

 protected:
    bool _warn_if_not_specialized = true;
};

}  // namespace mhp_planner

#endif  // SRC_OPTIMIZATION_INCLUDE_MHP_PLANNER_OPTIMIZATION_OPTIMIZATION_PROBLEM_INTERFACE_H_
