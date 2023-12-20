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

#ifndef UR_COLLISION_CONSTRAINT_MHP_PLANNER_H
#define UR_COLLISION_CONSTRAINT_MHP_PLANNER_H

#include <mhp_planner/core/factory.h>
#include <ur10_planner/ocp/constraints/ur_base_collision_constraint_corbo.h>

namespace mhp_planner {

class URCollisionConstraint : public URBaseCollisionConstraint
{
 public:
    using Ptr  = std::shared_ptr<URCollisionConstraint>;
    using UPtr = std::unique_ptr<URCollisionConstraint>;

    URCollisionConstraint() = default;

    URBaseCollisionConstraint::Ptr getInstance() const override;

    bool fromParameterServer(const std::string& ns) override;
};

FACTORY_REGISTER_OBJECT(URCollisionConstraint, URBaseCollisionConstraint)

}  // namespace mhp_planner

#endif  // UR_COLLISION_CONSTRAINT_MHP_PLANNER_H
