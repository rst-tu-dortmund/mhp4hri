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

#ifndef ROBOT_COST_FUNCTION_H
#define ROBOT_COST_FUNCTION_H

#include <mhp_robot/robot_kinematic/robot_kinematic.h>
#include <mhp_robot/robot_trajectory_optimization/robot_stage_preprocessor.h>
#include <Eigen/Core>

namespace mhp_robot {
namespace robot_trajectory_optimization {

class RobotCostFunction
{
 public:
    using Ptr  = std::shared_ptr<RobotCostFunction>;
    using UPtr = std::unique_ptr<RobotCostFunction>;

    RobotCostFunction() = default;

    RobotCostFunction(const RobotCostFunction&)            = delete;
    RobotCostFunction(RobotCostFunction&&)                 = default;
    RobotCostFunction& operator=(const RobotCostFunction&) = delete;
    RobotCostFunction& operator=(RobotCostFunction&&)      = default;
    virtual ~RobotCostFunction() {}

    bool initialize(RobotStagePreprocessor::Ptr preprocessor, robot_kinematic::RobotKinematic::UPtr robot_kinematic);

    virtual double computeStateCost(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& x_ref,
                                    const Eigen::Ref<const Eigen::VectorXd>& s_ref);

    virtual double computeControlCost(const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_ref,
                                      const Eigen::Ref<const Eigen::VectorXd>& s_ref);

    virtual double computeControlDeviationCost(const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_ref,
                                               const Eigen::Ref<const Eigen::VectorXd>& u_prev);

    virtual double computeStateControlCost(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                           const Eigen::Ref<const Eigen::VectorXd>& x_ref, const Eigen::Ref<const Eigen::VectorXd>& u_ref,
                                           const Eigen::Ref<const Eigen::VectorXd>& s_ref);

    virtual void computeStateCostGradient(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& x_ref,
                                          const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::VectorXd> dx);

    virtual void computeControlCostGradient(const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_ref,
                                            const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::VectorXd> du);

    virtual void computeControlDeviationCostGradient(const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_prev,
                                                     const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::VectorXd> du);

    virtual void computeStateControlCostGradient(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                                 const Eigen::Ref<const Eigen::VectorXd>& x_ref, const Eigen::Ref<const Eigen::VectorXd>& u_ref,
                                                 const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::VectorXd> dx,
                                                 Eigen::Ref<Eigen::VectorXd> du);

    virtual void computeStateCostHessian(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& x_ref,
                                         const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::MatrixXd> dxdx);

    virtual void computeControlCostHessian(const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_ref,
                                           const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::MatrixXd> dudu);

    virtual void computeControlDeviationCostHessian(const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_prev,
                                                    const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::MatrixXd> dudu);

    virtual void computeStateControlCostHessian(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                                const Eigen::Ref<const Eigen::VectorXd>& x_ref, const Eigen::Ref<const Eigen::VectorXd>& u_ref,
                                                const Eigen::Ref<const Eigen::VectorXd>& s_ref, Eigen::Ref<Eigen::MatrixXd> dxdx,
                                                Eigen::Ref<Eigen::MatrixXd> dxdu, Eigen::Ref<Eigen::MatrixXd> dudu);

    virtual void update();

    virtual int getStateCostDimension() const;
    virtual int getControlCostDimension() const;
    virtual int getControlDeviationCostDimension() const;
    virtual int getStateControlCostDimension() const;

    void setR(const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& R);

    bool isInitialized() const;

    void setPlannerId(const int id)
    {
        _planner_id = id;
        if (_planner_id != 0) _ms_planner_mode = true;
    }
    int getPlannerId() const { return _planner_id; }
    bool isPlannerSet() const { return _ms_planner_mode; }

 protected:
    using RobotKinematic = robot_kinematic::RobotKinematic;

    RobotStagePreprocessor::Ptr _preprocessor;
    RobotKinematic::UPtr _robot_kinematic;

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> _R_diag;
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> _Rd_diag;

    bool _evaluate_controls          = true;
    bool _evaluate_control_deviation = false;
    bool _initialized                = false;
    bool _use_preprocessor           = false;

    // Variables for Multistage Planner
    int _planner_id       = 0;
    bool _ms_planner_mode = false;
};

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot

#endif  // ROBOT_COST_FUNCTION_H
