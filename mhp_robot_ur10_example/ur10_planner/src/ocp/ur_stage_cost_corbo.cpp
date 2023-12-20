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

#include <ur10_planner/ocp/ur_stage_cost_corbo.h>
#include <ur10_planner/ocp/ur_stage_preprocessor_corbo.h>

namespace mhp_planner {

StageCost::Ptr URStageCost::getInstance() const { return std::make_shared<URStageCost>(); }

int URStageCost::getNonIntegralStateTermDimension(int k) const { return _cost_function->getStateCostDimension(); }

int URStageCost::getNonIntegralControlTermDimension(int k) const { return _cost_function->getControlCostDimension(); }

int URStageCost::getNonIntegralStateControlTermDimension(int k) const { return _cost_function->getStateControlCostDimension(); }

int URStageCost::getNonIntegralControlDeviationTermDimension(int k) const { return _cost_function->getControlDeviationCostDimension(); }

void URStageCost::computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const
{
    assert(cost.cols() == getNonIntegralStateTermDimension(k));

    if (_collision_potential)
    {
        cost << _cost_function->computeStateCost(x_k, _x_ref->getReferenceCached(k), _s_ref->getReferenceCached(k)) +
                    _collision_potential->computePotentials(k, x_k);
    }
    else
    {
        cost << _cost_function->computeStateCost(x_k, _x_ref->getReferenceCached(k), _s_ref->getReferenceCached(k));
    }
}

void URStageCost::computeNonIntegralControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& u_k, Eigen::Ref<Eigen::VectorXd> cost) const
{
    assert(cost.cols() == getNonIntegralControlTermDimension(k));

    cost << _cost_function->computeControlCost(u_k, _u_ref->getReferenceCached(k), Eigen::Matrix<double, 1, 1>::Zero());
}

void URStageCost::computeNonIntegralStateControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k,
                                                     const Eigen::Ref<const Eigen::VectorXd>& u_k, Eigen::Ref<Eigen::VectorXd> cost) const
{
    assert(cost.cols() == getNonIntegralStateControlTermDimension(k));

    cost << _cost_function->computeStateControlCost(x_k, u_k, _x_ref->getReferenceCached(k), _u_ref->getReferenceCached(k),
                                                    _s_ref->getReferenceCached(k));
}

void URStageCost::computeNonIntegralControlDeviationTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                                         const Eigen::Ref<const Eigen::VectorXd>& u_prev, double dt,
                                                         Eigen::Ref<Eigen::VectorXd> cost) const
{
    assert(cost.cols() == getNonIntegralControlDeviationTermDimension(k));

    cost << _cost_function->computeControlDeviationCost(u_k, _u_ref->getReferenceCached(k), u_prev);
}

bool URStageCost::update(int n, double t, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref, ReferenceTrajectoryInterface* sref,
                         bool single_dt, const Eigen::VectorXd& x0, StagePreprocessor::Ptr stage_preprocessor, const std::vector<double>& dts,
                         const DiscretizationGridInterface*)
{
    _x_ref = &xref;
    _u_ref = &uref;

    if (sref)
    {
        _s_ref = sref;
    }

    if (_cost_function && !_cost_function->isInitialized())
    {
        URStagePreprocessor::Ptr preprocessor = std::dynamic_pointer_cast<URStagePreprocessor>(stage_preprocessor);
        _cost_function->initialize(preprocessor ? preprocessor->getPreprocessor() : nullptr, std::make_unique<URKinematic>());
    }
    if (_collision_potential && !_collision_potential->isInitialized())
    {
        URStagePreprocessor::Ptr preprocessor = std::dynamic_pointer_cast<URStagePreprocessor>(stage_preprocessor);
        _collision_potential->initialize(preprocessor ? preprocessor->getPreprocessor() : nullptr, std::make_unique<URCollision>());
    }

    if (!single_dt || (int)dts.size() > 1)
    {
        PRINT_WARNING_NAMED("Multiple dt currently not supported! Using first dt.");
    }
    if (_collision_potential) _collision_potential->update(dts[0]);

    return false;
}

bool URStageCost::checkParameters(int state_dim, int control_dim) const
{
    bool success = true;

    if (!_cost_function)
    {
        PRINT_ERROR("URStageCosts: No cost function was selected.");
        success = false;
    }

    return success;
}
void URStageCost::setPlannerId(const int id)
{
    if (_cost_function) _cost_function->setPlannerId(id);
    if (_collision_potential) _collision_potential->setPlannerId(id);
}

int URStageCost::getPlannerId() const
{
    if (_cost_function && _collision_potential)
    {
        if (_cost_function->getPlannerId() == _collision_potential->getPlannerId())
        {
            return _collision_potential->getPlannerId();
        }
        else
        {
            ROS_ERROR("URStageCost: Multistage Planner ID not identical");
            return -1;
        };
    }
    else if (_cost_function)
    {
        ROS_WARN("URStageCost: Multistage Planner ID not defined for collision potential");
        return _cost_function->getPlannerId();
    }
    else if (_collision_potential)
    {
        ROS_WARN("URStageCost: Multistage Planner ID not defined for cost function");
        return _collision_potential->getPlannerId();
    }
    else
    {
        ROS_ERROR("URStageCost: Multistage ID error");
        return -1;
    };
}

bool URStageCost::isPlannerSet() const
{
    if (_cost_function && _collision_potential)
    {
        return (_cost_function->isPlannerSet() && _collision_potential->isPlannerSet());
    }
    else if (_cost_function)
    {
        ROS_WARN_ONCE("URStageCost: Multistage Planner not set for undefined collision potential");
        return _cost_function->isPlannerSet();
    }
    else if (_collision_potential)
    {
        ROS_WARN_ONCE("URStageCost: Multistage Planner not set for undefined cost function");
        return _collision_potential->isPlannerSet();
    }
    else
    {
        ROS_ERROR("URFinalStageCost: undefined potential and cost function");
        return false;
    };
}

bool URStageCost::fromParameterServer(const std::string& ns)
{
    ros::NodeHandle nh;

    // Set cost function
    std::string stage_cost_function_type;
    nh.getParam(ns + "/cost_function/stage_cost_function_type", stage_cost_function_type);
    if (!stage_cost_function_type.empty())
    {
        /* code */

        _cost_function = Factory<URBaseCostFunction>::instance().create(stage_cost_function_type);
        // import parameters
        if (_cost_function)
        {
            if (!_cost_function->fromParameterServer(ns + "/cost_function")) return false;
        }
        else
        {
            ROS_ERROR("URStageCosts: unknown cost function specified.");
            return false;
        }
    }
    else
    {
        _cost_function = {};
        return false;
    }

    // Set collision potential
    std::string collision_potential_type;
    nh.getParam(ns + "/collision_potential/collision_potential_type", collision_potential_type);
    if (collision_potential_type == "None")
    {
        _collision_potential = {};
    }
    else if (collision_potential_type == "URCollisionPotentialMohri")
    {
        _collision_potential = Factory<URBaseCollisionPotential>::instance().create(collision_potential_type);
        // import parameters
        if (_collision_potential)
        {
            if (!_collision_potential->fromParameterServer(ns + "/collision_potential")) return false;
        }
        else
        {
            ROS_ERROR("URStageCosts: unknown collision potential specified.");
            return false;
        }
    }
    else
    {
        _collision_potential = {};
    }

    return true;
}

}  // namespace mhp_planner
