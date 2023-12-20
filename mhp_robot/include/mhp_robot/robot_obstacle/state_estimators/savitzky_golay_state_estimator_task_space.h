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

#ifndef SAVITZKY_GOLAY_STATE_ESTIMATOR_TASK_SPACE_H
#define SAVITZKY_GOLAY_STATE_ESTIMATOR_TASK_SPACE_H

#ifdef SG
#include <gram_savitzky_golay/gram_savitzky_golay.h>
#endif  // SG

#include <mhp_robot/robot_obstacle/state_estimators/base_state_estimator_task_space.h>
namespace mhp_robot {
namespace robot_obstacle {
namespace state_estimators {

#ifdef SG
class SavitzkyGolayStateEstimatorTaskSpace : public BaseStateEstimatorTaskSpace
{
 public:
    using Ptr  = std::shared_ptr<SavitzkyGolayStateEstimatorTaskSpace>;
    using UPtr = std::unique_ptr<SavitzkyGolayStateEstimatorTaskSpace>;

    SavitzkyGolayStateEstimatorTaskSpace(int id, const std::vector<int>& vel_filter_parameters = {4, 2, 1},
                                         const std::vector<int>& acc_filter_parameters = {15, 2, 2});

    void predict(double t) override;
    void update(const Eigen::Ref<const Eigen::Matrix4d>& pose, double t) override;

    void setF(const Eigen::Ref<const Eigen::Matrix<double, 18, 18>>& F);
    void setInitialState(const Eigen::Ref<const Eigen::Matrix<double, 18, 1>>& state);

 private:
    void allocateMemory();
    void extractState();
    void configureFilter(gram_sg::SavitzkyGolayFilter& filter, const int& filter_window_size, const int& filter_order, const int& filter_init_point,
                         const int& filter_derivation_order);
    // internal states
    std::vector<Eigen::VectorXd> _values_pos;
    std::vector<Eigen::VectorXd> _values_rot;

    Eigen::VectorXd _values_x;
    Eigen::VectorXd _values_y;
    Eigen::VectorXd _values_z;
    Eigen::VectorXd _values_px;
    Eigen::VectorXd _values_py;
    Eigen::VectorXd _values_pz;

    Eigen::VectorXd _time;

    Eigen::Matrix<double, 18, 1> _x;
    Eigen::Matrix<double, 18, 18> _F;

    int _window_size = 5;
    int _num_values  = 0;

    int _vel_filter_window_size      = 60;
    int _vel_filter_order            = 2;
    int _vel_filter_init_point       = _vel_filter_window_size;
    int _vel_filter_derivation_order = 1;

    int _acc_filter_window_size      = 36;
    int _acc_filter_order            = 3;
    int _acc_filter_init_point       = _acc_filter_window_size;
    int _acc_filter_derivation_order = 0;

    gram_sg::SavitzkyGolayFilter _vel_filter;
    gram_sg::SavitzkyGolayFilter _acc_filter;
};
#else   // SG
class SavitzkyGolayStateEstimatorTaskSpace : public BaseStateEstimatorTaskSpace
{
 public:
    using Ptr  = std::shared_ptr<SavitzkyGolayStateEstimatorTaskSpace>;
    using UPtr = std::unique_ptr<SavitzkyGolayStateEstimatorTaskSpace>;

    SavitzkyGolayStateEstimatorTaskSpace(int id, const std::vector<int>& vel_filter_parameters = {4, 2, 1},
                                         const std::vector<int>& acc_filter_parameters = {15, 2, 2})
        : BaseStateEstimatorTaskSpace(id)
    {
        ROS_ERROR_ONCE("SavitzkyGolayStateEstimatorTaskSpace is not available. Please install the SavitzkyGolayFilter package.");
    };

    void predict(double t) override{};
    void update(const Eigen::Ref<const Eigen::Matrix4d>& pose, double t) override{};
};
#endif  // SG
}  // namespace state_estimators
}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // SAVITZKY_GOLAY_STATE_ESTIMATOR_TASK_SPACE_H
