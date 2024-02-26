/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  This software is currently not released.
 *  Redistribution and use in source and binary forms,
 *  with or without modification, are prohibited.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Maximilian Krämer
 *********************************************************************/

#ifndef EUCLIDEAN_SETPOINT_OBJECTIVE_H
#define EUCLIDEAN_SETPOINT_OBJECTIVE_H

#include <mhp_robot/robot_setpoint_manager/objectives/base_setpoint_objective.h>

namespace mhp_robot {
namespace robot_set_point_manager {
namespace objectives {

class EuclideanSetpointObjective : public BaseSetpointObjective
{
 public:
    using Ptr  = std::shared_ptr<EuclideanSetpointObjective>;
    using UPtr = std::unique_ptr<EuclideanSetpointObjective>;

    EuclideanSetpointObjective(const Eigen::Ref<const Eigen::VectorXd>& ref);

    void setReferenceState(const Eigen::Ref<const Eigen::VectorXd>& ref);

    double calculateObjective(const Eigen::Ref<const Eigen::VectorXd>& current, const Eigen::Ref<const Eigen::VectorXd>& next) const override;

 private:
    Eigen::VectorXd _ref;
};

}  // namespace objectives
}  // namespace robot_set_point_manager
}  // namespace mhp_robot

#endif  // EUCLIDEAN_SETPOINT_OBJECTIVE_H
