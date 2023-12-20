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

#ifndef FORECAST_UNCERTAINTY_PROCESS_H
#define FORECAST_UNCERTAINTY_PROCESS_H

#include <mhp_robot/robot_misc/gmm.h>
#include <mhp_robot/robot_obstacle/obstacle_core.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/base_process.h>

namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

class ForecastUncertaintyProcess : public BaseProcess
{
 public:
    using Ptr  = std::shared_ptr<ForecastUncertaintyProcess>;
    using UPtr = std::unique_ptr<ForecastUncertaintyProcess>;

    ForecastUncertaintyProcess(const std::string& name, const int& rate, const double& extrapolation_dt, const int& extrapolation_steps,
                               const int& gmm_samples, const int& gmm_components, const int& gmm_dimensions, const bool split_skeleton,
                               const std::string& mode, const bool gmm_weighting);

    bool process(std::map<int, robot_misc::Obstacle>& static_obstacles, std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                 std::map<int, robot_misc::Human>& humans, std::map<int, robot_misc::UtilityObject>& utility_objects,
                 std::map<int, robot_misc::Plane>& planes, bool forced = false) override;
    bool initialize() override;

#ifdef ARMADILLO
    void trainGMM(const int& joint);
    void getVariance();
#endif  // ARMADILLO

 protected:
 private:
    int _n_joints       = 11;
    int _buffer_counter = 0;
    bool _buffer_full   = false;
    const int _rate     = 25;

    const double _extrapolation_dt = 0.1;
    const int _extrapolation_steps = 30;

    std::vector<GMM> _gmm;
    const int _gmm_samples      = 37;
    const int _gmm_components   = 4;
    const int _gmm_dimensions   = 2;
    const int _gmm_update_steps = 37;
    int _gmm_joint_counter      = 0;
    const bool _gmm_weighting   = false;

    const int _limit_times_sigma  = 2;
    const int _add_times_sigma    = 1;
    const double _radius_increase = 1.1;

#ifdef ARMADILLO
    uvec _split_idx;
    uvec _old_split_idx;

    const vec _dependency_vector = {0, 0, 0, 2, 3, 4, 0, 6, 7, 8, 0};
#endif  // ARMADILLO

    const std::vector<std::string> body_part_names{"Hip", "Neck", "Head", "LUArm", "LFArm", "LHand", "RUArm", "RFArm", "RHand", "Leg"};

    Eigen::MatrixXd _gt_angle_buffer;
    std::vector<std::vector<Eigen::Vector<double, 11>>> _extrapolation_angles_buffer;
    std::vector<Eigen::MatrixXd> _error_gt_extrapolation;

    void errorUpdateBuffer(std::pair<const int, robot_misc::Human>& human);
    Eigen::VectorXd resample(const Eigen::Ref<const Eigen::VectorXd> input);
    std::map<int, robot_misc::Obstacle> _additional_body_parts;

    bool _split_skeleton = false;  // split skeleton builds up new body parts if true and grows the radius if false

    enum mode {
        Gmm,
        Constant
    } _mode = Gmm;  // GMM: Calculates online GMM based on previous results to decide whether a uncertainty estimation is required. Constant: Assumes
                    // a strict growing uncertainty only useful for radius Increase

    double _time = 0.0;
};
}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot
#endif  // FORECAST_UNCERTAINTY_PROCESS_H
