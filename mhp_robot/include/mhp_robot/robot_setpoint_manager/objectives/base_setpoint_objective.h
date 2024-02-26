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

#ifndef BASE_SETPOINT_OBJECTIVE_H
#define BASE_SETPOINT_OBJECTIVE_H

#include <Eigen/Dense>
#include <memory>

namespace mhp_robot {
namespace robot_set_point_manager {
namespace objectives {

class BaseSetpointObjective
{
 public:
    using Ptr  = std::shared_ptr<BaseSetpointObjective>;
    using UPtr = std::unique_ptr<BaseSetpointObjective>;

    BaseSetpointObjective() = default;

    BaseSetpointObjective(const BaseSetpointObjective&) = delete;
    BaseSetpointObjective(BaseSetpointObjective&&)      = default;
    BaseSetpointObjective& operator=(const BaseSetpointObjective&) = delete;
    BaseSetpointObjective& operator=(BaseSetpointObjective&&) = default;
    virtual ~BaseSetpointObjective() {}

    virtual double calculateObjective(const Eigen::Ref<const Eigen::VectorXd>& current, const Eigen::Ref<const Eigen::VectorXd>& next) const;
};

}  // namespace objectives
}  // namespace robot_set_point_manager
}  // namespace mhp_robot

#endif  // BASE_SETPOINT_OBJECTIVE_H
