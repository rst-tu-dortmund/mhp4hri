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

#include <ur10_planner/ocp/ur_inequality_constraint_corbo.h>
#include <ur10_planner/ocp/ur_stage_preprocessor_corbo.h>

namespace mhp_planner {

StageInequalityConstraint::Ptr URInequalityConstraint::getInstance() const { return std::make_shared<URInequalityConstraint>(); }

int URInequalityConstraint::getNonIntegralStateTermDimension(int k) const { return _collision_constraint ? _dimension : 0; }

int URInequalityConstraint::getNonIntegralControlDeviationTermDimension(int k) const
{
    return _acceleration_constraint ? _acceleration_constraint->getDimension() : 0;
}

void URInequalityConstraint::computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const
{
    if (_collision_constraint) _collision_constraint->compute(k, x_k, cost);
}

void URInequalityConstraint::computeNonIntegralControlDeviationTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                                                    const Eigen::Ref<const Eigen::VectorXd>& u_prev, double dt,
                                                                    Eigen::Ref<Eigen::VectorXd> cost) const
{
    if (_acceleration_constraint) _acceleration_constraint->compute(u_k, u_prev, dt, cost);
}

bool URInequalityConstraint::update(int n, double t, mhp_planner::ReferenceTrajectoryInterface& xref, mhp_planner::ReferenceTrajectoryInterface& uref,
                                    mhp_planner::ReferenceTrajectoryInterface* sref, bool single_dt, const Eigen::VectorXd& x0,
                                    StagePreprocessor::Ptr stage_preprocessor, const std::vector<double>& dts, const DiscretizationGridInterface*)
{
    // If necessary initialize
    if (_collision_constraint && !_collision_constraint->isInitialized())
    {
        URStagePreprocessor::Ptr preprocessor = std::dynamic_pointer_cast<URStagePreprocessor>(stage_preprocessor);
        _collision_constraint->initialize(preprocessor ? preprocessor->getPreprocessor() : nullptr, std::make_unique<URCollision>());
    }

    if (!single_dt || (int)dts.size() > 1)
    {
        PRINT_WARNING_NAMED("Multiple dt currently not supported! Using first dt.");
    }

    int dimension = 0;
    if (_collision_constraint)
    {
        _collision_constraint->update(dts[0]);
        dimension = _collision_constraint->getDimension();
    }

    if (dimension != _dimension)
    {
        _dimension = dimension;
        return true;
    }

    return false;
}

bool URInequalityConstraint::fromParameterServer(const std::string& ns)
{
    ros::NodeHandle nh;

    // acceleration constraint
    std::string acceleration_constraint_type;
    nh.getParam(ns + "/acceleration_constraint/ur_acceleration_constraint_type", acceleration_constraint_type);
    if (acceleration_constraint_type == "None")
    {
        _acceleration_constraint = {};
    }
    else if (acceleration_constraint_type == "URAccelerationConstraint")
    {
        _acceleration_constraint = Factory<URBaseAccelerationConstraint>::instance().create(acceleration_constraint_type);
        if (_acceleration_constraint)
        {
            if (!_acceleration_constraint->fromParameterServer(ns + "/acceleration_constraint")) return false;
        }
        else
        {
            PRINT_ERROR("URInequalityConstraints: unknown acceleration_constraints specified.");
            return false;
        }
    }
    else
    {
        PRINT_ERROR("URInequalityConstraint: Unknown acceleration constraint type!");
        return false;
    }

    // collision constraint
    std::string collision_constraint_type;
    nh.getParam(ns + "/collision_constraint/ur_collision_constraint_type", collision_constraint_type);
    if (collision_constraint_type == "None")
    {
        _collision_constraint = {};
    }
    else if (collision_constraint_type == "URCollisionConstraint")
    {

        _collision_constraint = Factory<URBaseCollisionConstraint>::instance().create(collision_constraint_type);

        // import parameters
        if (_collision_constraint)
        {
            if (!_collision_constraint->fromParameterServer(ns + "/collision_constraint")) return false;
        }
        else
        {
            PRINT_ERROR("URInequalityConstraints: unknown collision constraint specified.");
            return false;
        }
    }
    else
    {
        PRINT_ERROR("URInequalityConstraint: Unknown collision constraint type!");
        return false;
    }

    return true;
}

}  // namespace mhp_planner
