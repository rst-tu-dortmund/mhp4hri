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

#ifndef SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_FUNCTIONS_STAGE_PREPROCESSOR_H_
#define SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_FUNCTIONS_STAGE_PREPROCESSOR_H_

#include <mhp_planner/core/factory.h>
#include <mhp_planner/core/reference_trajectory.h>

#include <Eigen/Core>

namespace mhp_planner {

class DiscretizationGridInterface;

class StagePreprocessor
{
 public:
    using Ptr      = std::shared_ptr<StagePreprocessor>;
    using ConstPtr = std::shared_ptr<const StagePreprocessor>;

    StagePreprocessor();

    virtual Ptr getInstance() const                                         = 0;
    virtual void precompute(const Eigen::Ref<const Eigen::VectorXd>& input) = 0;
    virtual bool update(int n, double t, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref, ReferenceTrajectoryInterface* sref,
                        bool single_dt, const Eigen::VectorXd& x0, const std::vector<double>& dts, const DiscretizationGridInterface* /*grid*/)
    {
        return true;
    }

    std::vector<Eigen::VectorXd> _vector_data;
    std::vector<Eigen::MatrixXd> _matrix_data;

    virtual bool fromParameterServer(const std::string& ns) = 0;
};

using StagePreprocessorFactory = Factory<StagePreprocessor>;
#define FACTORY_REGISTER_STAGE_PREPROCESSOR(type) FACTORY_REGISTER_OBJECT(type, StagePreprocessor)

}  // namespace mhp_planner

#endif  // SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_FUNCTIONS_STAGE_PREPROCESSOR_H_
