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

#include <mhp_robot/robot_obstacle/obstacle_pipeline/forecast_uncertainty_process.h>
namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

ForecastUncertaintyProcess::ForecastUncertaintyProcess(const std::string& name, const int& rate, const double& extrapoaltion_dt,
                                                       const int& extrapolation_steps, const int& gmm_samples, const int& gmm_components,
                                                       const int& gmm_dimensions, const bool split_skeleton, const std::string& mode,
                                                       const bool gmm_weighting)
    : BaseProcess(name),
      _rate(rate),
      _extrapolation_dt(extrapoaltion_dt),
      _extrapolation_steps(extrapolation_steps),
      _gmm_samples(gmm_samples),
      _gmm_components(gmm_components),
      _gmm_dimensions(gmm_dimensions),
      _gmm_weighting(gmm_weighting),
      _split_skeleton(split_skeleton)
{
    if (mode.compare("GMM") == 0)
        _mode = ForecastUncertaintyProcess::Gmm;
    else if (mode.compare("Constant") == 0)
        _mode = ForecastUncertaintyProcess::Constant;
    else if (mode.compare("None") == 0)
        return;
    else
        ROS_ERROR("ForecastUncertaintyProcess: Ivnvalid uncertainty mode");
}

bool ForecastUncertaintyProcess::process(std::map<int, robot_misc::Obstacle>& static_obstacles,
                                         std::map<int, robot_misc::Obstacle>& dynamic_obstacles, std::map<int, robot_misc::Human>& humans,
                                         std::map<int, robot_misc::UtilityObject>& utility_objects, std::map<int, robot_misc::Plane>& planes,
                                         bool forced)
{
    if (!_initialized)
    {
        ROS_WARN("ForecastUncertaintyProcess: Cannot process in an uninitialized state.");
        return false;
    }

    if (_active || forced)
    {
        _buffer_counter++;
        // Iterate over Humans
        for (auto& human : humans)
        {
            // Update modes inside Human (for sending to Cobra)
            human.second._skeleton_splitting = _split_skeleton;
            human.second._uncertainty_mode   = _mode;

            // Start Uncertainty estimation
#ifdef ARMADILLO
            if (_mode == Gmm)
            {
                // std::chrono::steady_clock::time_point start2 = std::chrono::steady_clock::now();

                // buffer current angles
                _gt_angle_buffer.block(0, 0, _gt_angle_buffer.rows(), _gt_angle_buffer.cols() - 1) =
                    _gt_angle_buffer.block(0, 1, _gt_angle_buffer.rows(), _gt_angle_buffer.cols() - 1);
                Eigen::Vector<double, 11> tmp(human.second._joint_angles.data());  // Transform to EIGEN
                _gt_angle_buffer.col(_gt_angle_buffer.cols() - 1) = tmp;

                // buffer extrapolation angles
                _extrapolation_angles_buffer.erase(_extrapolation_angles_buffer.begin());
                _extrapolation_angles_buffer.push_back(human.second._all_extrapolated_angles);

                // calculate error between GT and Extrapolation
                errorUpdateBuffer(human);

                // Check if buffer counter is high enough to start with GMM and only do it every _gmm_update_steps
                if (_buffer_counter >= 2 * _gmm_samples && !_buffer_full)  // 2 * _gmm_samples to secure that the buffer is full and has meaningfull
                                                                           // values instead of the first extrapolation errors
                {
                    _buffer_full = true;
                }

                int trainFlag = 0;
                if (_gmm_update_steps == 0)
                {
                    trainFlag = 0;
                }
                else
                {
                    trainFlag = _buffer_counter % _gmm_update_steps;
                }

                if (_buffer_full && trainFlag == 0)
                {

                    if (_gmm_joint_counter < 11)
                    {
                        // std::cout << "GMM Joint Counter: " << _gmm_joint_counter << std::endl;
                        trainGMM(_gmm_joint_counter);
                        _gmm.at(_gmm_joint_counter).get_variance();
                        _buffer_counter = _buffer_counter - 1;
                        _gmm_joint_counter++;
                        if (_gmm_joint_counter == 11 && _gmm_update_steps == 0)
                        {
                            _gmm_joint_counter = 0;
                        }
                    }
                    else
                    {
                        _gmm_joint_counter = 0;
                        // _buffer_counter    = _buffer_counter;
                    }

                    // Time Measurement

                    // if (_gmm_joint_counter < 11)
                    // {
                    //     // std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
                    //     // std::cout << "Calculation Time GMM Train: "
                    //     //           << std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2).count() << "[µs]" << std::endl;
                    //     // _time += std::chrono::duration<double, std::micro>(end2 - start2).count();
                    //     if (_gmm_joint_counter == 10)
                    //     {
                    //         //                                std::cout << "Calculation Time GMM Train (for all joints together,
                    //         //                                splitted to different iterations): " << _time
                    //         //                                          << "[µs]" << std::endl;
                    //         _time = 0.0;
                    //     }
                    // }
                }

                // Add uncertainty to human model
                vec stdLimits(human.second._joint_angles.size());
                int secChoice = 5;
                for (int joint = 0; joint < human.second._joint_angles.size(); ++joint)
                {
                    stdLimits(joint) = _limit_times_sigma * sqrt(_gmm.at(joint)._sigma_y.at(secChoice));
                }
                _split_idx = find(stdLimits >= 1.0);

                // std::chrono::steady_clock::time_point start3 = std::chrono::steady_clock::now();
                if (_split_skeleton)
                {
                    ROS_INFO_ONCE("Forecast Uncertainty Process: Use skeleton splitting for uncertainty representation");
                    if (_buffer_full && !(_split_idx.is_empty()))
                    {
                        // Clean up "old uncertainties"
                        human.second.deleteUncertaintyObstacles();

                        // Preallocate Vector for all angle Variants for all _extrapolation_steps in the future
                        std::vector<std::vector<Eigen::MatrixXd>> allAngleVariants(_extrapolation_steps);

                        // Test Matrix
                        Eigen::MatrixXd tmp;

                        // Iterate over extrapolation steps in the future
                        for (int timeStep = 0; timeStep < _extrapolation_steps - 1; ++timeStep)
                        {
                            // Resize angle matrix to default size of joint angle vector
                            allAngleVariants.at(timeStep).resize(human.second._joint_angles.size());

                            // Iterate over all joint angles to check if uncertainty split is necessary
                            for (int joint = 0; joint < human.second._joint_angles.size(); ++joint)
                            {
                                // For first joint just add elements of extrapolated angles for corresponding time step
                                if (joint == 0)
                                {
                                    allAngleVariants.at(timeStep).at(joint).conservativeResize(
                                        human.second._all_extrapolated_angles.at(timeStep).rows(), 1);
                                    allAngleVariants.at(timeStep).at(joint).col(0) = human.second._all_extrapolated_angles.at(timeStep);
                                }
                                // For the rest of the joints add elements of parent joint angles for corresponding time step
                                else
                                {
                                    allAngleVariants.at(timeStep).at(joint).conservativeResize(
                                        allAngleVariants.at(timeStep).at(_dependency_vector.at(joint)).rows(),
                                        allAngleVariants.at(timeStep).at(_dependency_vector.at(joint)).cols());
                                    allAngleVariants.at(timeStep).at(joint) = allAngleVariants.at(timeStep).at(_dependency_vector.at(joint));
                                }
                                // Check if split is required
                                if (any(_split_idx == joint))
                                {
                                    // std::cout << "Split for joint: " << joint << std::endl;
                                    // std::cout << allAngleVariants.at(timeStep).size() << std::endl;
                                    // std::cout << "LEngth: " << allAngleVariants.at(timeStep).at(_dependency_vector.at(joint)).cols() << std::endl;
                                    // Iterate for all angle variants for corresponding joint and time step
                                    int dependency_vector_length = allAngleVariants.at(timeStep).at(_dependency_vector.at(joint)).cols();
                                    for (int varCount = 0; varCount < dependency_vector_length; varCount++)
                                    {
                                        // std::cout << "start: " << allAngleVariants.at(timeStep).at(joint) << std::endl;
                                        // std::cout << "VarCount: " << varCount << std::endl;
                                        // Build up uncertainty to the top and bottom
                                        Eigen::Vector<double, 11> tmp1 = allAngleVariants.at(timeStep).at(_dependency_vector.at(joint)).col(varCount);
                                        Eigen::Vector<double, 11> tmp2 = allAngleVariants.at(timeStep).at(_dependency_vector.at(joint)).col(varCount);
                                        tmp1(joint)                    = tmp1(joint) + _add_times_sigma * sqrt(_gmm.at(joint)._sigma_y(timeStep));
                                        tmp2(joint)                    = tmp2(joint) - _add_times_sigma * sqrt(_gmm.at(joint)._sigma_y(timeStep));
                                        // Check joint limitations
                                        human.second.checkJointLimits(tmp1);
                                        human.second.checkJointLimits(tmp2);

                                        // Add uncertainty angle vectors to angle matrix
                                        allAngleVariants.at(timeStep).at(joint).conservativeResize(
                                            allAngleVariants.at(timeStep).at(joint).rows(), allAngleVariants.at(timeStep).at(joint).cols() + 2);
                                        allAngleVariants.at(timeStep).at(joint).col(allAngleVariants.at(timeStep).at(joint).cols() - 2) = tmp1;
                                        allAngleVariants.at(timeStep).at(joint).col(allAngleVariants.at(timeStep).at(joint).cols() - 1) = tmp2;
                                        // std::cout << "End: " << allAngleVariants.at(timeStep).at(joint) << std::endl;
                                    }
                                }
                            }
                            // Add Uncertainty states to obstacle descriptions
                            std::vector<int> splitVec = conv_to<std::vector<int>>::from(_split_idx);
                            human.second.addUncertaintyObstacles(allAngleVariants.at(timeStep), timeStep + 1, splitVec);
                        }
                    }
                    else if (_buffer_full && (_split_idx.is_empty()))
                    {
                        //  Delete all uncertainty states if no uncertainty split is required
                        human.second.deleteUncertaintyObstacles();
                    }
                }
                else
                {
                    ROS_INFO_ONCE("Forecast Uncertainty Process: Use body part radius increase for uncertainty representation");
                    if (_buffer_full && !(_split_idx.is_empty()))
                    {
                        Eigen::VectorXd body_splits_idx = Eigen::VectorXd::Zero(human.second._body_part_count);
                        for (auto var : _split_idx)
                        {
                            switch (var)
                            {
                                // Hip joint --> Neck Body part
                                case 0:
                                    body_splits_idx[1]++;
                                    break;
                                // Neck joint --> Head Body part
                                case 1:
                                    body_splits_idx[2]++;
                                    break;

                                // Left Shoulder joints --> LUArm+LFArm+LHand
                                case 2:
                                    body_splits_idx[3]++;  // LUArm
                                    body_splits_idx[4]++;  // LFArm
                                    body_splits_idx[5]++;  // LHand
                                    break;
                                case 3:
                                    body_splits_idx[3]++;  // LUArm
                                    body_splits_idx[4]++;  // LFArm
                                    body_splits_idx[5]++;  // LHand
                                    break;
                                case 4:
                                    body_splits_idx[3]++;  // LUArm
                                    body_splits_idx[4]++;  // LFArm
                                    body_splits_idx[5]++;  // LHand
                                    break;

                                // Left Elbow joints --> LFArm+LHand
                                case 5:
                                    body_splits_idx[4]++;  // LFArm
                                    body_splits_idx[5]++;  // LHand
                                    break;
                                // Right Shoulder joints --> RUArm+RFArm+RHand
                                case 6:
                                    body_splits_idx[6]++;  // RUArm
                                    body_splits_idx[7]++;  // RFArm
                                    body_splits_idx[8]++;  // RHand
                                    break;
                                case 7:
                                    body_splits_idx[6]++;  // RUArm
                                    body_splits_idx[7]++;  // RFArm
                                    body_splits_idx[8]++;  // RHand
                                    break;
                                case 8:
                                    body_splits_idx[6]++;  // RUArm
                                    body_splits_idx[7]++;  // RFArm
                                    body_splits_idx[8]++;  // RHand
                                    break;

                                // Rigth Elbow joints --> RFArm+RHand
                                case 9:
                                    body_splits_idx[7]++;  // LFArm
                                    body_splits_idx[8]++;  // LHand
                                    break;

                                // Leg joints --> Leg
                                case 10:
                                    body_splits_idx[0]++;  // Hip
                                    body_splits_idx[9]++;  // Leg
                                    break;
                                default:
                                    ROS_ERROR("Forecast_uncertainty_process : Invalid joint number uncertainty estimation");
                            }
                        }
                        for (int bp = 0; bp < body_splits_idx.size(); bp++)
                        {
                            if (body_splits_idx(bp) > 0)
                            {
                                human.second._body_parts.find(body_part_names[bp])->second.bounding_box.future_radius.resize(_extrapolation_steps);
                                for (size_t cnt = 0; cnt < _extrapolation_steps; cnt++)
                                {
                                    human.second._body_parts.find(body_part_names[bp])->second.bounding_box.future_radius.at(cnt) =
                                        human.second._body_parts.find(body_part_names[bp])->second.bounding_box.radius *
                                        (1.0 + (body_splits_idx[bp] * (cnt + 1.0)) / 100.0);
                                }
                            }
                        }
                    }
                    else if (_buffer_full && (_split_idx.is_empty()))
                    {
                        for (auto& bp : human.second._body_parts)
                        {
                            bp.second.bounding_box.future_radius.clear();
                            //                        bp.second.bounding_box.radius = human.second._radius.find(bp.second.name +
                            //                        "_radius")->second;
                        }
                    }
                }
                // std::chrono::steady_clock::time_point end3 = std::chrono::steady_clock::now();
                // std::cout << "Calculation Time Uncertainty addition or skeleton split: "
                //           << std::chrono::duration_cast<std::chrono::microseconds>(end3 - start2).count() << "[µs]" << std::endl;
            }

#endif  // ARMADILLO

            if (_mode == Constant)
            {
                // Increase each body part radius by 1% or each extrapolation step into the future
                for (int bp = 0; bp < human.second._body_parts.size(); bp++)
                {
                    human.second._body_parts.find(body_part_names[bp])->second.bounding_box.future_radius.resize(_extrapolation_steps);
                    for (size_t cnt = 0; cnt < _extrapolation_steps; cnt++)
                    {
                        human.second._body_parts.find(body_part_names[bp])->second.bounding_box.future_radius.at(cnt) =
                            human.second._body_parts.find(body_part_names[bp])->second.bounding_box.radius * (1.0 + ((cnt + 1.0)) / 100.0);
                    }
                }
            }
        }
    }
    return true;
}
bool ForecastUncertaintyProcess::initialize()
{
#ifdef ARMADILLO
    if (_mode == Gmm)
    {
        // Init GMMs for each joint
        _gmm.resize(_n_joints);
        for (int j = 0; j < _n_joints; ++j)
        {
            _gmm.at(j) = GMM(_gmm_components, _gmm_samples, _gmm_dimensions, _extrapolation_steps,
                             _gmm_weighting);  // GMM with 4 Components, 60 Observations for each update, and 2 dimesions
                                               // (time and error)
        }

        // Init buffers
        _gt_angle_buffer.resize(
            _n_joints, static_cast<int>(_extrapolation_steps * _extrapolation_dt * _rate + 1));  // Save all GT angles for full extrapolation length
        _extrapolation_angles_buffer.resize(static_cast<int>(_extrapolation_steps * _extrapolation_dt * _rate + 1));
        _error_gt_extrapolation.resize(_gmm_samples);
    }
#else   // ARMADILLO
    if (_mode == Gmm)
    {
        ROS_ERROR_ONCE("Forecast Uncertainty Process: GMM mode not available without Armadillo. No uncertainty estimation performed.");
    }
#endif  // ARMADILLO

    _initialized = true;

    return true;
}

#ifdef ARMADILLO
void ForecastUncertaintyProcess::trainGMM(const int& joint)
{
    //    std::cout << "Size Error Vector: " << _error_gt_extrapolation.size() << std::endl;

    arma::vec tmp(_gmm_samples * _extrapolation_steps);
    int start = 0;
    int end   = _extrapolation_steps - 1;
    for (auto j = 0; j < _error_gt_extrapolation.size(); ++j)
    {
        arma::vec tmp2(_error_gt_extrapolation.at(j).col(joint).data(), _extrapolation_steps, true,
                       false);          // Transform to arma vector
        tmp.subvec(start, end) = tmp2;  // attach all errors behind each other
        start                  = end + 1;
        end                    = end + _extrapolation_steps;
    }
    _gmm.at(joint).update_GMM(&tmp, joint);
}
#endif  // ARMADILLO
void ForecastUncertaintyProcess::errorUpdateBuffer(std::pair<const int, robot_misc::Human>& human)
{
    // Note different buffer frequencies --> Current Angles with _rate and extrapolations with 1/_extrapolation_dt -->
    // Interpolation
    Eigen::MatrixXd error(static_cast<int>(_extrapolation_steps), human.second._joint_angles.size());
    for (int jointNum = 0; jointNum < human.second._joint_angles.size(); ++jointNum)
    {
        Eigen::VectorXd resampled_gt_angles;
        resampled_gt_angles =
            resample(_gt_angle_buffer.row(jointNum).tail(_extrapolation_steps * _extrapolation_dt * _rate));  // Last 3 seconds GT angles
        Eigen::VectorXd extrap_angles(static_cast<int>(_extrapolation_steps));

        for (auto j = 0; j < _extrapolation_angles_buffer.front().size();
             ++j)  // First Element in Buffert is the extrapolation over the last 3 seconds (passed now)
        {
            extrap_angles(j) = _extrapolation_angles_buffer.front().at(j)(jointNum);
        };

        error.col(jointNum) = resampled_gt_angles - extrap_angles;  // Error for the extrapolation that ends at this sample.
    }
    _error_gt_extrapolation.erase(_error_gt_extrapolation.begin());
    _error_gt_extrapolation.push_back(error);
}

Eigen::VectorXd ForecastUncertaintyProcess::resample(const Eigen::Ref<const Eigen::VectorXd> input)
{
    const int input_size  = static_cast<int>(input.size());
    const int output_size = static_cast<int>((input_size - 1) * (1 / _extrapolation_dt) / _rate) + 1;

    Eigen::VectorXi Index(30);
    Index << 1, 3, 6, 8, 11, 13, 16, 18, 21, 23, 26, 28, 31, 33, 36, 38, 41, 43, 46, 48, 51, 53, 56, 58, 61, 63, 66, 68, 71, 73;

    Eigen::VectorXd weights(30);
    weights << 0.5, 1, 0.5, 1, 0.5, 1, 0.5, 1, 0.5, 1, 0.5, 1, 0.5, 1, 0.5, 1, 0.5, 1, 0.5, 1, 0.5, 1, 0.5, 1, 0.5, 1, 0.5, 1, 0.5, 1;

    Eigen::VectorXd input_vec = input;

    Eigen::VectorXd output_vec(output_size);
    for (int i = 0; i <= output_size - 1; ++i)
    {
        output_vec(i) = input_vec(Index(i)) + weights(i) * (input_vec(Index(i) + 1) - input_vec(Index(i)));
    }

    return output_vec;
}

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot
