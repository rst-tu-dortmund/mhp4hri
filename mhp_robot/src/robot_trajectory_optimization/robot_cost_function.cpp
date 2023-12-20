/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2023,
 *  TU Dortmund University, Institute of Control Theory and System Engineering
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
*  Authors: Maximilian Krämer
*  Maintainer(s)/Modifier(s): Heiko Renz
 *********************************************************************/

#include <mhp_robot/robot_trajectory_optimization/robot_cost_function.h>

namespace mhp_robot {
namespace robot_trajectory_optimization {

bool RobotCostFunction::initialize(RobotStagePreprocessor::Ptr preprocessor, RobotKinematic::UPtr robot_kinematic)
{
    _robot_kinematic = std::move(robot_kinematic);

    if (preprocessor)
    {
        _preprocessor = preprocessor;
    }
    else
    {
        if (_use_preprocessor) ROS_WARN("RobotCostFunction: No preprocessor provided. No preprocessor is used!");
        _use_preprocessor = false;
    }

    _initialized = true;

    return true;
}

bool RobotCostFunction::isInitialized() const { return _initialized; }

double RobotCostFunction::computeStateCost(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& x_ref,
                                           const Eigen::Ref<const Eigen::VectorXd>& s_ref)
{
    return 0.0;
}

double RobotCostFunction::computeControlCost(const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_ref,
                                             const Eigen::Ref<const Eigen::VectorXd>& s_ref)
{
    if (!_evaluate_controls) return 0.0;

    double cost        = 0.0;
    Eigen::VectorXd ud = u_k - u_ref;

    cost = ud.transpose() * _R_diag * ud;

    return cost;
}

double RobotCostFunction::computeControlDeviationCost(const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_ref,
                                                      const Eigen::Ref<const Eigen::VectorXd>& u_prev)
{
    if (!_evaluate_control_deviation) return 0.0;

    double cost        = 0.0;
    Eigen::VectorXd ud = u_k - u_prev;

    cost = ud.transpose() * _Rd_diag * ud;

    return cost;
}

double RobotCostFunction::computeStateControlCost(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                                  const Eigen::Ref<const Eigen::VectorXd>& x_ref, const Eigen::Ref<const Eigen::VectorXd>& u_ref,
                                                  const Eigen::Ref<const Eigen::VectorXd>& s_ref)
{
    return 0.0;
}

void RobotCostFunction::computeStateCostGradient(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& x_ref,
                                                 const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::VectorXd> dx)
{
    dx.setZero();
}

void RobotCostFunction::computeControlCostGradient(const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_ref,
                                                   const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::VectorXd> du)
{
    du = 2 * (u_k - u_ref).transpose() * _R_diag;
}

void RobotCostFunction::computeControlDeviationCostGradient(const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                                            const Eigen::Ref<const Eigen::VectorXd>& u_prev,
                                                            const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::VectorXd> du)
{
    du = 2 * (u_k - u_prev).transpose() * _Rd_diag;
}

void RobotCostFunction::computeStateControlCostGradient(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                                        const Eigen::Ref<const Eigen::VectorXd>& x_ref,
                                                        const Eigen::Ref<const Eigen::VectorXd>& u_ref,
                                                        const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::VectorXd> dx,
                                                        Eigen::Ref<Eigen::VectorXd> du)
{
    dx.setZero();
    du.setZero();
}

void RobotCostFunction::computeStateCostHessian(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& x_ref,
                                                const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::MatrixXd> dxdx)
{
    dxdx.setZero();
}

void RobotCostFunction::computeControlCostHessian(const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_ref,
                                                  const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::MatrixXd> dudu)
{
    dudu = 2 * _R_diag;
}

void RobotCostFunction::computeControlDeviationCostHessian(const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                                           const Eigen::Ref<const Eigen::VectorXd>& u_prev,
                                                           const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::MatrixXd> dudu)
{
    dudu = 2 * _Rd_diag;
}

void RobotCostFunction::computeStateControlCostHessian(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                                       const Eigen::Ref<const Eigen::VectorXd>& x_ref, const Eigen::Ref<const Eigen::VectorXd>& u_ref,
                                                       const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::MatrixXd> dxdx,
                                                       Eigen::Ref<Eigen::MatrixXd> dxdu, Eigen::Ref<Eigen::MatrixXd> dudu)
{
    dxdx.setZero();
    dxdu.setZero();
    dudu.setZero();
}

void RobotCostFunction::update() {}

int RobotCostFunction::getStateCostDimension() const { return 0; }

int RobotCostFunction::getControlCostDimension() const { return _evaluate_controls ? 1 : 0; }

int RobotCostFunction::getControlDeviationCostDimension() const { return _evaluate_control_deviation ? 1 : 0; }

int RobotCostFunction::getStateControlCostDimension() const { return 0; }

void RobotCostFunction::setR(const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& R) { _R_diag = R; }

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot
