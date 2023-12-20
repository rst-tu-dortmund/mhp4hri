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

#ifndef UR_ACCELERATION_CONSTRAINT_MHP_PLANNER_H
#define UR_ACCELERATION_CONSTRAINT_MHP_PLANNER_H

#include <mhp_planner/core/factory.h>
#include <ur10_planner/ocp/constraints/ur_base_acceleration_constraint_corbo.h>

namespace mhp_planner {

class URAccelerationConstraint : public URBaseAccelerationConstraint
{
 public:
    using Ptr  = std::shared_ptr<URAccelerationConstraint>;
    using UPtr = std::unique_ptr<URAccelerationConstraint>;

    URAccelerationConstraint() = default;

    URBaseAccelerationConstraint::Ptr getInstance() const override;

    bool fromParameterServer(const std::string& ns) override;
};
FACTORY_REGISTER_OBJECT(URAccelerationConstraint, URBaseAccelerationConstraint)

}  // namespace mhp_planner

#endif  // UR_ACCELERATION_CONSTRAINT_MHP_PLANNER_H
