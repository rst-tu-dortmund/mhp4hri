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

#ifndef SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_FUNCTIONS_FINAL_STATE_CONSTRAINTS_H_
#define SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_FUNCTIONS_FINAL_STATE_CONSTRAINTS_H_

// #include <mhp_planner/ocp/functions/final_state_cost.h>

#include <mhp_planner/core/reference_trajectory.h>
#include <mhp_planner/ocp/functions/stage_functions.h>
#include <mhp_planner/systems/system_dynamics_interface.h>

#include <memory>

namespace mhp_planner {

class TerminalBall : public FinalStageConstraint
{
 public:
    using Ptr      = std::shared_ptr<TerminalBall>;
    using ConstPtr = std::shared_ptr<const TerminalBall>;

    TerminalBall() = default;

    TerminalBall(const Eigen::Ref<const Eigen::MatrixXd>& S, double gamma)
    {
        setWeightS(S);
        setGamma(gamma);
    }

    FinalStageConstraint::Ptr getInstance() const override { return std::make_shared<TerminalBall>(); }

    bool isEqualityConstraint() const override { return false; }

    int getNonIntegralStateTermDimension(int k) const override { return 1; }

    bool setWeightS(const Eigen::Ref<const Eigen::MatrixXd>& S);
    bool setWeightS(const Eigen::DiagonalMatrix<double, -1>& S);
    const Eigen::MatrixXd& getWeightS() const { return _S; }

    void setGamma(double gamma) { _gamma = gamma; }
    double getGamma() { return _gamma; }

    void computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const override;

    bool update(int n, double t, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref, ReferenceTrajectoryInterface* sref,
                bool single_dt, const Eigen::VectorXd& x0, FinalStageCost::ConstPtr final_stage_cost, StagePreprocessor::Ptr stage_preprocessor,
                const std::vector<double>& dts, const DiscretizationGridInterface* /*grid*/) override
    {
        _x_ref = &xref;

        _zero_x_ref = _x_ref->isZero();

        return false;
    }

    bool checkParameters(int state_dim, int control_dim, FinalStageCost::ConstPtr final_stage_cost) const override;

 protected:
    Eigen::MatrixXd _S;
    Eigen::DiagonalMatrix<double, -1> _S_diag;
    double _gamma = 0.0;

    const ReferenceTrajectoryInterface* _x_ref = nullptr;
    bool _zero_x_ref                           = false;

    bool _diagonal_mode               = false;
    bool _diagonal_mode_intentionally = false;
};

}  // namespace mhp_planner

#endif  // SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_FUNCTIONS_FINAL_STATE_CONSTRAINTS_H_
