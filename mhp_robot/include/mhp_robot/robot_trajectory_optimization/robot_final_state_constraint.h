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

#ifndef ROBOT_FINAL_STATE_CONSTRAINT_H
#define ROBOT_FINAL_STATE_CONSTRAINT_H

#include <mhp_robot/robot_trajectory_optimization/robot_stage_preprocessor.h>
#include <Eigen/Core>
#include <memory>

namespace mhp_robot {
namespace robot_trajectory_optimization {

class RobotFinalStateConstraint
{
 public:
    using Ptr  = std::shared_ptr<RobotFinalStateConstraint>;
    using UPtr = std::unique_ptr<RobotFinalStateConstraint>;

    RobotFinalStateConstraint() = default;

    RobotFinalStateConstraint(const RobotFinalStateConstraint&)            = delete;
    RobotFinalStateConstraint(RobotFinalStateConstraint&&)                 = default;
    RobotFinalStateConstraint& operator=(const RobotFinalStateConstraint&) = delete;
    RobotFinalStateConstraint& operator=(RobotFinalStateConstraint&&)      = default;
    virtual ~RobotFinalStateConstraint() {}

    bool initialize(RobotStagePreprocessor::Ptr preprocessor);

    virtual void compute(const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& s_ref,
                         Eigen::Ref<Eigen::VectorXd> cost) = 0;
    virtual int getDimension() const                       = 0;

    bool isInitialized() const;

    void setPlannerId(const int id)
    {
        _planner_id = id;
        if (_planner_id != 0) _ms_planner_mode = true;
    }
    int getPlannerId() const { return _planner_id; }
    bool isPlannerSet() const { return _ms_planner_mode; }

 protected:
    RobotStagePreprocessor::Ptr _preprocessor;

    bool _initialized      = false;
    bool _use_preprocessor = false;

    // Variables for Multistage Planner
    int _planner_id       = 0;
    bool _ms_planner_mode = false;
};

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot

#endif  // ROBOT_FINAL_STATE_CONSTRAINT_H
