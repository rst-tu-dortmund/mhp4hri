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

#ifndef SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_STRUCTURED_OCP_DISCRETIZATION_GRIDS_FINITE_DIFFERENCES_GRID_H_
#define SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_STRUCTURED_OCP_DISCRETIZATION_GRIDS_FINITE_DIFFERENCES_GRID_H_

#include <mhp_planner/ocp/structured_ocp/discretization_grids/full_discretization_grid_base.h>

#include <memory>

namespace mhp_planner {

class FiniteDifferencesGrid : public FullDiscretizationGridBase
{
 public:
    using Ptr  = std::shared_ptr<FiniteDifferencesGrid>;
    using UPtr = std::unique_ptr<FiniteDifferencesGrid>;

    FiniteDifferencesGrid()          = default;
    virtual ~FiniteDifferencesGrid() = default;

    //! Return a newly created shared instance of the implemented class
    DiscretizationGridInterface::Ptr getInstance() const override { return std::make_shared<FiniteDifferencesGrid>(); }

    //! Get access to the associated factory
    static Factory<DiscretizationGridInterface>& getFactory() { return Factory<DiscretizationGridInterface>::instance(); }

    void fromParameterServer(const std::string& ns) override;

 protected:
    void createEdges(NlpFunctions& nlp_fun, OptimizationEdgeSet& edges, SystemDynamicsInterface::Ptr dynamics) override;

    bool isDtFixedIntended() const override { return true; }
};

FACTORY_REGISTER_DISCRETIZATION_GRID(FiniteDifferencesGrid)

}  // namespace mhp_planner

#endif  // SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_STRUCTURED_OCP_DISCRETIZATION_GRIDS_FINITE_DIFFERENCES_GRID_H_
