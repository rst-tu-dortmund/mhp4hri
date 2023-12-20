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

#ifndef ROBOT_QUADRATIC_COST_JOINT_SPACE_H
#define ROBOT_QUADRATIC_COST_JOINT_SPACE_H

#include <mhp_robot/robot_trajectory_optimization/robot_cost_function.h>

namespace mhp_robot {
namespace robot_trajectory_optimization {

class RobotQuadraticCostJointSpace : virtual public RobotCostFunction
{
 public:
    using Ptr  = std::shared_ptr<RobotQuadraticCostJointSpace>;
    using UPtr = std::unique_ptr<RobotQuadraticCostJointSpace>;

    RobotQuadraticCostJointSpace() = default;

    double computeStateCost(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& x_ref,
                            const Eigen::Ref<const Eigen::VectorXd>& s_ref) override;

    void computeStateCostGradient(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& x_ref,
                                  const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::VectorXd> dx) override;

    void computeStateCostHessian(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& x_ref,
                                 const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::MatrixXd> dxdx) override;

    int getStateCostDimension() const override;

    void setQ(const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& Q);

 protected:
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> _Q_diag;

    bool _evaluate_states = true;
};

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot

#endif  // ROBOT_QUADRATIC_COST_JOINT_SPACE_H
