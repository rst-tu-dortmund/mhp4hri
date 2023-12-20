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

#include <ur10_planner/ocp/potentials/ur_collision_potential_mohri_corbo.h>

namespace mhp_planner {

mhp_planner::URBaseCollisionPotential::Ptr URCollisionPotentialMohri::getInstance() const { return std::make_shared<URCollisionPotentialMohri>(); }

bool URCollisionPotentialMohri::fromParameterServer(const std::string& ns)
{
    ros::NodeHandle nh;

    // Set weigths and margins for the different obstacle classes
    std::vector<double> weights, margins;
    nh.getParam(ns + "/weights", weights);
    nh.getParam(ns + "/margins", margins);

    _weights = Eigen::Map<const Eigen::VectorXd>(weights.data(), weights.size());
    _margins = Eigen::Map<const Eigen::VectorXd>(margins.data(), margins.size());

    // Get parameters which elements are considered
    nh.getParam(ns + "/roof_collision", _roof_collision);
    nh.getParam(ns + "/ground_collision", _ground_collision);
    nh.getParam(ns + "/self_collision", _self_collision);
    nh.getParam(ns + "/static_obstacles", _static_obstacles);
    nh.getParam(ns + "/dynamic_obstacles", _dynamic_obstacles);
    nh.getParam(ns + "/plane_collision", _plane_collision);
    nh.getParam(ns + "/human_head_collision", _human_head_collision);
    nh.getParam(ns + "/human_trunk_collision", _human_trunk_collision);
    nh.getParam(ns + "/human_limbs_collision", _human_limbs_collision);

    // Parameter for extended Mohri Potential
    nh.getParam(ns + "/extended", _extended);

    // Set parameters for additional options
    nh.getParam(ns + "/use_preprocessor", _use_preprocessor);
    nh.getParam(ns + "/min_distances", _min_distances);

    // Set Parameter if obstacle predictions are used and if the planning delay of the planner is considered as offset)
    nh.getParam(ns + "/obstacle_prediction", _obstacle_prediction);
    nh.getParam(ns + "/prediction_offset", _prediction_offest);

    // Set parameters for uncertainty consideration of human motion extrapolations
    nh.getParam(ns + "/uncertainty_estimation_human", _uncertainty_estimation_human);
    std::string uncertainty_representation_mode;
    std::string uncertainty_estimation_mode = "None";
    nh.getParam("/extrapolation_uncertainties/uncertainty_mode", uncertainty_estimation_mode);
    nh.getParam(ns + "/uncertainty_mode", uncertainty_representation_mode);
    if (uncertainty_representation_mode == "SkeletonSplitting" && uncertainty_estimation_mode == "GMM")
    {
        _uncertainty_mode = SkeletonSplitting;
    }
    else if (uncertainty_representation_mode == "SkeletonSplitting" && uncertainty_estimation_mode == "Constant")
    {
        PRINT_WARNING("URCollisionPotentialMohri: Uncertainty representation is set to RadiusIncrease due to Constant estimation!");
        _uncertainty_mode = RadiusIncrease;
    }
    else if (uncertainty_representation_mode == "SkeletonSplitting" && uncertainty_estimation_mode == "None")
    {
        PRINT_WARNING("URCollisionPotentialMohri: Uncertainty representation is deactivated due to no uncertainty estimation!");
        _uncertainty_mode             = NoUncertaintyEstimation;
        _uncertainty_estimation_human = false;
    }
    else if (uncertainty_representation_mode == "RadiusIncrease" &&
             (uncertainty_estimation_mode == "GMM" || uncertainty_estimation_mode == "Constant"))
    {
        _uncertainty_mode = RadiusIncrease;
    }
    else if (uncertainty_representation_mode == "RadiusIncrease" && uncertainty_estimation_mode == "None")
    {
        PRINT_WARNING("URCollisionPotentialMohri: Uncertainty representation is deactivated due to no uncertainty estimation!");
        _uncertainty_mode             = NoUncertaintyEstimation;
        _uncertainty_estimation_human = false;
    }
    else if (uncertainty_representation_mode == "None")
    {
        PRINT_WARNING("URCollisionPotentialMohri: Uncertainty estimation is disabled since mode None is chosen!");
        _uncertainty_mode             = NoUncertaintyEstimation;
        _uncertainty_estimation_human = false;
    }
    else
    {
        PRINT_ERROR("URCollisionPotentialMohri: Unknown uncertainty mode!");
        return false;
    }

    return true;
}

}  // namespace mhp_planner
