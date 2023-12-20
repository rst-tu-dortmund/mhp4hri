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

#include <mhp_robot/MsgDistances.h>
#include <mhp_robot/robot_misc/common.h>
#include <mhp_robot/robot_obstacle/obstacle_core.h>
#include <ros/ros.h>
#include <limits>

using ObstaclePublisher = mhp_robot::robot_obstacle::ObstaclePublisher;
using ObstacleProcessor = mhp_robot::robot_obstacle::ObstacleProcessor;
using ObstacleTrajectoryGenerator = mhp_robot::robot_obstacle::ObstacleTrajectoryGenerator;

using BaseStateEstimatorTaskSpace = mhp_robot::robot_obstacle::state_estimators::BaseStateEstimatorTaskSpace;
using KalmanStateEstimatorTaskSpace = mhp_robot::robot_obstacle::state_estimators::KalmanStateEstimatorTaskSpace;
using PolynomStateEstimatorTaskSpace = mhp_robot::robot_obstacle::state_estimators::PolynomStateEstimatorTaskSpace;
using SavitzkyGolayEstimatorTaskSpace = mhp_robot::robot_obstacle::state_estimators::SavitzkyGolayStateEstimatorTaskSpace;

using BaseStateEstimatorJointSpace = mhp_robot::robot_obstacle::state_estimators::BaseStateEstimatorJointSpace;
using KalmanStateEstimatorJointSpace = mhp_robot::robot_obstacle::state_estimators::KalmanStateEstimatorJointSpace;
using PolynomStateEstimatorJointSpace = mhp_robot::robot_obstacle::state_estimators::PolynomStateEstimatorJointSpace;
using SavitzkyGolayEstimatorJointSpace = mhp_robot::robot_obstacle::state_estimators::SavitzkyGolayStateEstimatorJointSpace;

using BaseProcess = mhp_robot::robot_obstacle::obstacle_pipeline::BaseProcess;
using LoadObstacleProcess = mhp_robot::robot_obstacle::obstacle_pipeline::LoadObstacleProcess;
using ManualObstacleSimulatorProcess = mhp_robot::robot_obstacle::obstacle_pipeline::ManualObstacleSimulatorProcess;
using AutomaticObstacleSimulatorProcess = mhp_robot::robot_obstacle::obstacle_pipeline::AutomaticObstacleSimulatorProcess;
using TFObstacleProcess = mhp_robot::robot_obstacle::obstacle_pipeline::TFObstacleProcess;
using StateEstimationProcess = mhp_robot::robot_obstacle::obstacle_pipeline::StateEstimationProcess;
using StateExtrapolationProcess = mhp_robot::robot_obstacle::obstacle_pipeline::StateExtrapolationProcess;
using MotionClassificationProcess = mhp_robot::robot_obstacle::obstacle_pipeline::MotionClassificationProcess;
using WorkspaceFilterProcess = mhp_robot::robot_obstacle::obstacle_pipeline::WorkspaceFilterProcess;
using HumanMotionPredictionProcess = mhp_robot::robot_obstacle::obstacle_pipeline::HumanMotionPredictionProcess;
using ForecastUncertaintyProcess = mhp_robot::robot_obstacle::obstacle_pipeline::ForecastUncertaintyProcess;

void configurePolynomFilterTaskSpace(PolynomStateEstimatorTaskSpace &filter, double rate)
{
    double dt = 1 / rate;

    Eigen::Matrix<double, 9, 9> F;
    Eigen::Matrix<double, 18, 18> Fg;
    F.setIdentity();
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    F(3, 6) = dt;
    F(4, 7) = dt;
    F(5, 8) = dt;

    F(0, 6) = 0.5 * dt * dt;
    F(1, 7) = 0.5 * dt * dt;
    F(2, 8) = 0.5 * dt * dt;

    Fg.setIdentity();
    Fg.block<9, 9>(0, 0) = F;
    Fg.block<9, 9>(9, 9) = F;

    filter.setF(Fg);
}

void configureKalmanFilterTaskSpace(KalmanStateEstimatorTaskSpace &filter, double rate)
{
    double dt = 1 / rate;

    Eigen::Matrix<double, 3, 3> R;
    R = Eigen::Matrix<double, 3, 3>::Identity() * 1e-15;

    Eigen::Matrix<double, 9, 9> Q;
    Eigen::Matrix<double, 1, 9> q1, q2, q3;

    q1 << (dt * dt * dt * dt * dt * dt) / 36.0, (dt * dt * dt * dt * dt * dt) / 36.0, (dt * dt * dt * dt * dt * dt) / 36.0,
        (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt) / 6.0,
        (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0;

    q2 << (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt) / 4.0,
        (dt * dt * dt * dt) / 4.0, (dt * dt * dt * dt) / 4.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0;

    q3 << (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0,
        (dt * dt * dt) / 2.0, (dt * dt), (dt * dt), (dt * dt);

    Q.block<1, 9>(0, 0) = q1;
    Q.block<1, 9>(1, 0) = q1;
    Q.block<1, 9>(2, 0) = q1;
    Q.block<1, 9>(3, 0) = q2;
    Q.block<1, 9>(4, 0) = q2;
    Q.block<1, 9>(5, 0) = q2;
    Q.block<1, 9>(6, 0) = q3;
    Q.block<1, 9>(7, 0) = q3;
    Q.block<1, 9>(8, 0) = q3;

    Q = Q * 0.0001;
    // Q = Q * 0;

    Eigen::Matrix<double, 9, 9> F;
    F.setIdentity();
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    F(3, 6) = dt;
    F(4, 7) = dt;
    F(5, 8) = dt;

    F(0, 6) = 0.5 * dt * dt;
    F(1, 7) = 0.5 * dt * dt;
    F(2, 8) = 0.5 * dt * dt;

    Eigen::Matrix<double, 3, 9> H;
    H.setZero();
    H(0, 0) = 1;
    H(1, 1) = 1;
    H(2, 2) = 1;

    Eigen::Matrix<double, 9, 9> P;
    P = Eigen::Matrix<double, 9, 9>::Identity() * 1e-5;

    filter.setF(F);
    filter.setH(H);
    filter.setQ(Q);
    filter.setR(R);
    filter.setInitialP(P);
}

#ifdef SG
void configureGolayFilterTaskSpace(SavitzkyGolayEstimatorTaskSpace &filter, double rate)
{
    double dt = 1 / rate;

    Eigen::Matrix<double, 9, 9> F;
    Eigen::Matrix<double, 18, 18> Fg;
    F.setIdentity();
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    F(3, 6) = dt;
    F(4, 7) = dt;
    F(5, 8) = dt;

    F(0, 6) = 0.5 * dt * dt;
    F(1, 7) = 0.5 * dt * dt;
    F(2, 8) = 0.5 * dt * dt;

    Fg.setIdentity();
    Fg.block<9, 9>(0, 0) = F;
    Fg.block<9, 9>(9, 9) = F;

    filter.setF(Fg);
}
#endif // SG
void configurePolynomFilterJointSpace(PolynomStateEstimatorJointSpace &filter, double rate)
{
    double dt = 1 / rate;

    Eigen::Matrix<double, 33, 33> F;

    F.setIdentity();
    F(0, 11) = dt;
    F(1, 12) = dt;
    F(2, 13) = dt;
    F(3, 14) = dt;
    F(4, 15) = dt;
    F(5, 16) = dt;
    F(6, 17) = dt;
    F(7, 18) = dt;
    F(8, 19) = dt;
    F(9, 20) = dt;
    F(10, 21) = dt;
    F(11, 22) = dt;
    F(12, 23) = dt;
    F(13, 24) = dt;
    F(14, 25) = dt;
    F(15, 26) = dt;
    F(16, 27) = dt;
    F(17, 28) = dt;
    F(18, 29) = dt;
    F(19, 30) = dt;
    F(20, 31) = dt;
    F(21, 32) = dt;

    F(0, 22) = 0.5 * dt * dt;
    F(1, 23) = 0.5 * dt * dt;
    F(2, 24) = 0.5 * dt * dt;
    F(3, 25) = 0.5 * dt * dt;
    F(4, 26) = 0.5 * dt * dt;
    F(5, 27) = 0.5 * dt * dt;
    F(6, 28) = 0.5 * dt * dt;
    F(7, 29) = 0.5 * dt * dt;
    F(8, 30) = 0.5 * dt * dt;
    F(9, 31) = 0.5 * dt * dt;
    F(10, 32) = 0.5 * dt * dt;

    filter.setF(F);
}

void configureKalmanFilterJointSpace(KalmanStateEstimatorJointSpace &filter, double rate)
{
    double dt = 1 / rate;

    Eigen::Matrix<double, 11, 11> R;
    R = Eigen::Matrix<double, 11, 11>::Identity() * 1e-15;

    Eigen::Matrix<double, 33, 33> Q;
    Eigen::Matrix<double, 1, 33> q1, q2, q3;

    q1 << (dt * dt * dt * dt * dt * dt) / 36.0, (dt * dt * dt * dt * dt * dt) / 36.0, (dt * dt * dt * dt * dt * dt) / 36.0,
        (dt * dt * dt * dt * dt * dt) / 36.0, (dt * dt * dt * dt * dt * dt) / 36.0, (dt * dt * dt * dt * dt * dt) / 36.0,
        (dt * dt * dt * dt * dt * dt) / 36.0, (dt * dt * dt * dt * dt * dt) / 36.0, (dt * dt * dt * dt * dt * dt) / 36.0,
        (dt * dt * dt * dt * dt * dt) / 36.0, (dt * dt * dt * dt * dt * dt) / 36.0, (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0,
        (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0,
        (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0,
        (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0,
        (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0,
        (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0;

    q2 << (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0,
        (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0,
        (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt * dt) / 12.0, (dt * dt * dt * dt) / 4.0,
        (dt * dt * dt * dt) / 4.0, (dt * dt * dt * dt) / 4.0, (dt * dt * dt * dt) / 4.0, (dt * dt * dt * dt) / 4.0, (dt * dt * dt * dt) / 4.0,
        (dt * dt * dt * dt) / 4.0, (dt * dt * dt * dt) / 4.0, (dt * dt * dt * dt) / 4.0, (dt * dt * dt * dt) / 4.0, (dt * dt * dt * dt) / 4.0,
        (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0,
        (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0;

    q3 << (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0,
        (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0, (dt * dt * dt * dt) / 6.0,
        (dt * dt * dt * dt) / 6.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0,
        (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt * dt) / 2.0, (dt * dt),
        (dt * dt), (dt * dt), (dt * dt), (dt * dt), (dt * dt), (dt * dt), (dt * dt), (dt * dt), (dt * dt), (dt * dt);

    Q.block<1, 33>(0, 0) = q1;
    Q.block<1, 33>(1, 0) = q1;
    Q.block<1, 33>(2, 0) = q1;
    Q.block<1, 33>(3, 0) = q1;
    Q.block<1, 33>(4, 0) = q1;
    Q.block<1, 33>(5, 0) = q1;
    Q.block<1, 33>(6, 0) = q1;
    Q.block<1, 33>(7, 0) = q1;
    Q.block<1, 33>(8, 0) = q1;
    Q.block<1, 33>(9, 0) = q1;
    Q.block<1, 33>(10, 0) = q1;
    Q.block<1, 33>(11, 0) = q2;
    Q.block<1, 33>(12, 0) = q2;
    Q.block<1, 33>(13, 0) = q2;
    Q.block<1, 33>(14, 0) = q2;
    Q.block<1, 33>(15, 0) = q2;
    Q.block<1, 33>(16, 0) = q2;
    Q.block<1, 33>(17, 0) = q2;
    Q.block<1, 33>(18, 0) = q2;
    Q.block<1, 33>(19, 0) = q2;
    Q.block<1, 33>(20, 0) = q2;
    Q.block<1, 33>(21, 0) = q2;
    Q.block<1, 33>(22, 0) = q3;
    Q.block<1, 33>(23, 0) = q3;
    Q.block<1, 33>(24, 0) = q3;
    Q.block<1, 33>(25, 0) = q3;
    Q.block<1, 33>(26, 0) = q3;
    Q.block<1, 33>(27, 0) = q3;
    Q.block<1, 33>(28, 0) = q3;
    Q.block<1, 33>(29, 0) = q3;
    Q.block<1, 33>(30, 0) = q3;
    Q.block<1, 33>(31, 0) = q3;
    Q.block<1, 33>(32, 0) = q3;

    Q = Q * 1e-15;
    Eigen::Matrix<double, 33, 33> F;
    F.setIdentity();
    F(0, 11) = dt;
    F(1, 12) = dt;
    F(2, 13) = dt;
    F(3, 14) = dt;
    F(4, 15) = dt;
    F(5, 16) = dt;
    F(6, 17) = dt;
    F(7, 18) = dt;
    F(8, 19) = dt;
    F(9, 20) = dt;
    F(10, 21) = dt;
    F(11, 22) = dt;
    F(12, 23) = dt;
    F(13, 24) = dt;
    F(14, 25) = dt;
    F(15, 26) = dt;
    F(16, 27) = dt;
    F(17, 28) = dt;
    F(18, 29) = dt;
    F(19, 30) = dt;
    F(20, 31) = dt;
    F(21, 32) = dt;

    F(0, 22) = 0.5 * dt * dt;
    F(1, 23) = 0.5 * dt * dt;
    F(2, 24) = 0.5 * dt * dt;
    F(3, 25) = 0.5 * dt * dt;
    F(4, 26) = 0.5 * dt * dt;
    F(5, 27) = 0.5 * dt * dt;
    F(6, 28) = 0.5 * dt * dt;
    F(7, 29) = 0.5 * dt * dt;
    F(8, 30) = 0.5 * dt * dt;
    F(9, 31) = 0.5 * dt * dt;
    F(10, 32) = 0.5 * dt * dt;

    Eigen::Matrix<double, 11, 33> H;
    H.setZero();
    H(0, 0) = 1;
    H(1, 1) = 1;
    H(2, 2) = 1;
    H(3, 3) = 1;
    H(4, 4) = 1;
    H(5, 5) = 1;
    H(6, 6) = 1;
    H(7, 7) = 1;
    H(8, 8) = 1;
    H(9, 9) = 1;
    H(10, 10) = 1;

    Eigen::Matrix<double, 33, 33> P;
    P = Eigen::Matrix<double, 33, 33>::Identity() * 1e-10;

    filter.setF(F);
    filter.setH(H);
    filter.setQ(Q);
    filter.setR(R);
    filter.setInitialP(P);
}

#ifdef SG
void configureGolayFilterJointSpace(SavitzkyGolayEstimatorJointSpace &filter, double rate)
{
    double dt = 1 / rate;

    Eigen::Matrix<double, 33, 33> F;

    F.setIdentity();
    F(0, 11) = dt;
    F(1, 12) = dt;
    F(2, 13) = dt;
    F(3, 14) = dt;
    F(4, 15) = dt;
    F(5, 16) = dt;
    F(6, 17) = dt;
    F(7, 18) = dt;
    F(8, 19) = dt;
    F(9, 20) = dt;
    F(10, 21) = dt;
    F(11, 22) = dt;
    F(12, 23) = dt;
    F(13, 24) = dt;
    F(14, 25) = dt;
    F(15, 26) = dt;
    F(16, 27) = dt;
    F(17, 28) = dt;
    F(18, 29) = dt;
    F(19, 30) = dt;
    F(20, 31) = dt;
    F(21, 32) = dt;

    F(0, 22) = 0.5 * dt * dt;
    F(1, 23) = 0.5 * dt * dt;
    F(2, 24) = 0.5 * dt * dt;
    F(3, 25) = 0.5 * dt * dt;
    F(4, 26) = 0.5 * dt * dt;
    F(5, 27) = 0.5 * dt * dt;
    F(6, 28) = 0.5 * dt * dt;
    F(7, 29) = 0.5 * dt * dt;
    F(8, 30) = 0.5 * dt * dt;
    F(9, 31) = 0.5 * dt * dt;
    F(10, 32) = 0.5 * dt * dt;

    filter.setF(F);
}
#endif // SG
int main(int argc, char **argv)
{
#ifndef NDEBUG
    sleep(5);
#endif

    ros::init(argc, argv, "workspace_monitor");
    ros::NodeHandle n("~");

    const double rate = 25;
    ros::Rate loop(rate);

    const double extrapolationStepLength = 0.1;
    const double extrapolationHorizonSteps = 30;

    /***************************************************************************************************
        Parameter Extraction
    /**************************************************************************************************/
    std::string mode = "manual";
    n.getParam("workspace_monitor_mode", mode);

    std::string prediction_mode = "None";
    n.getParam("/human_motion_extrapolation/prediction_mode", prediction_mode);

#ifdef SG
#else
    if (prediction_mode.compare("Golay") == 0)
    {
        ROS_ERROR(
            "Workspace Monitor: Golay Filter is not available. Please change the prediction mode to Polynom or None or activate Savitzky Golay "
            "dependency.");
        return 0;
    }
#endif // SG

    int extrapolation_steps = 0;
    n.getParam("/human_motion_extrapolation/extrapolation_steps", extrapolation_steps);

    bool constant_velocity_model = false;
    n.getParam("/human_motion_extrapolation/constant_velocity_model", constant_velocity_model);

    int polynom_fit_order = 3;
    int polynom_fit_window = 7;
    std::vector<int> vel_filter_estimator_parameter{4, 2, 1};
    std::vector<int> acc_filter_estimator_parameter{15, 2, 2};

    // Extracting polynom parameters from Parameter Server
    if (prediction_mode.compare("Polynom") == 0)
    {
        n.getParam("/human_motion_extrapolation/state_estimator/Polynom/polynom_fit_order", polynom_fit_order);
        n.getParam("/human_motion_extrapolation/state_estimator/Polynom/polynom_fit_window", polynom_fit_window);
    }

    // Extracting golay parameters from Parameter Server
    if (prediction_mode.compare("Golay") == 0)
    {
        std::string vel_string;
        std::string acc_string;
        n.getParam("/human_motion_extrapolation/state_estimator/Golay/vel_filter_parameter", vel_string);
        n.getParam("/human_motion_extrapolation/state_estimator/Golay/acc_filter_parameter", acc_string);
        std::stringstream vel_parameters(vel_string);
        std::stringstream acc_parameters(acc_string);
        int cntVel = 0;
        int cntAcc = 0;
        while (vel_parameters.good())
        {
            std::string substr;
            getline(vel_parameters, substr, ',');
            vel_filter_estimator_parameter[cntVel] = (std::stoi(substr));
            ++cntVel;
        }
        while (acc_parameters.good())
        {
            std::string substr;
            getline(acc_parameters, substr, ',');
            acc_filter_estimator_parameter[cntAcc] = (std::stoi(substr));
            ++cntAcc;
        }
    }

    // Extracting Parameters for foot print prediction
    int foot_polynom_fit_order = 3;
    int foot_polynom_fit_window = 7;
    std::vector<int> foot_vel_filter_estimator_parameter{4, 2, 1};
    std::vector<int> foot_acc_filter_estimator_parameter{15, 2, 2};

    std::string foot_print_predicition_mode = "None"; // foot print prediction uses the same amount of steps as the human motion prediction
    n.getParam("/footprint_human_motion_extrapolation/foot_print_prediction_mode", foot_print_predicition_mode);

    // Extracting polynom parameters from Parameter Server for footprint prediction
    if (foot_print_predicition_mode.compare("Polynom") == 0)
    {
        n.getParam("/footprint_human_motion_extrapolation/state_estimator/Polynom/polynom_fit_order", foot_polynom_fit_order);
        n.getParam("/footprint_human_motion_extrapolation/state_estimator/Polynom/polynom_fit_window", foot_polynom_fit_window);
    }

    // Extracting golay parameters from Parameter Server for footprint prediction
    if (foot_print_predicition_mode.compare("Golay") == 0)
    {
        std::string vel_string;
        std::string acc_string;
        n.getParam("/footprint_human_motion_extrapolation/state_estimator/Golay/vel_filter_parameter", vel_string);
        n.getParam("/footprint_human_motion_extrapolation/state_estimator/Golay/acc_filter_parameter", acc_string);
        std::stringstream vel_parameters(vel_string);
        std::stringstream acc_parameters(acc_string);
        int cntVel = 0;
        int cntAcc = 0;
        while (vel_parameters.good())
        {
            std::string substr;
            getline(vel_parameters, substr, ',');
            foot_vel_filter_estimator_parameter[cntVel] = (std::stoi(substr));
            ++cntVel;
        }
        while (acc_parameters.good())
        {
            std::string substr;
            getline(acc_parameters, substr, ',');
            foot_acc_filter_estimator_parameter[cntAcc] = (std::stoi(substr));
            ++cntAcc;
        }
    }

    // Extracting uncertainty Parameters
    std::string uncertainty_mode = "None";
    n.getParam("/extrapolation_uncertainties/uncertainty_mode", uncertainty_mode);

    bool split_skeleton = true;
    n.getParam("/extrapolation_uncertainties/split_skeleton", split_skeleton);

    bool gmm_weighting = false;
    n.getParam("/extrapolation_uncertainties/gmm_weighting", gmm_weighting);
    int gmm_samples = 0;
    n.getParam("/extrapolation_uncertainties/gmm_samples", gmm_samples);
    int gmm_components = 0;
    n.getParam("/extrapolation_uncertainties/gmm_components", gmm_components);
    int gmm_dimensions = 0;
    n.getParam("/extrapolation_uncertainties/gmm_dimensions", gmm_dimensions);

    /***************************************************************************************************
        Initialize required processes/objects
    /**************************************************************************************************/
    ObstaclePublisher obstacle_publisher;

    // Create processes
    LoadObstacleProcess::UPtr load_obstacle_ptr = std::make_unique<LoadObstacleProcess>("LoadObstacle");
    ManualObstacleSimulatorProcess::UPtr manual_simulator_ptr = std::make_unique<ManualObstacleSimulatorProcess>("ManualSimulator");
    AutomaticObstacleSimulatorProcess::UPtr automatic_simulator_ptr =
        std::make_unique<AutomaticObstacleSimulatorProcess>("AutomaticSimulator", true, prediction_mode);
    TFObstacleProcess::UPtr tf_obstacle_ptr = std::make_unique<TFObstacleProcess>("TFObstacleLookup");
    StateEstimationProcess::UPtr state_estimation_ptr = std::make_unique<StateEstimationProcess>("StateEstimation");
    StateExtrapolationProcess::UPtr state_extrapolation_ptr = std::make_unique<StateExtrapolationProcess>(
        "StateExtrapolation", extrapolationStepLength, extrapolationHorizonSteps, extrapolation_steps, constant_velocity_model);
    MotionClassificationProcess::UPtr motion_classification_ptr =
        std::make_unique<MotionClassificationProcess>("MotionClassification", 0.2, std::numeric_limits<double>::max());
    WorkspaceFilterProcess::UPtr workspace_filter_ptr = std::make_unique<WorkspaceFilterProcess>("WorkspaceDistanceFilter", 2);
    HumanMotionPredictionProcess::UPtr human_prediction_ptr = std::make_unique<HumanMotionPredictionProcess>("HumanMotionPrediction");
    ForecastUncertaintyProcess::UPtr forecast_uncertainty_ptr =
        std::make_unique<ForecastUncertaintyProcess>("ForecastUncertainty", rate, extrapolationStepLength, extrapolationHorizonSteps, gmm_samples,
                                                     gmm_components, gmm_dimensions, split_skeleton, uncertainty_mode, gmm_weighting);

    /***************************************************************************************************
        Define state estimators for obstacles
    /**************************************************************************************************/
    // Each obstacle has an individual state estimator --> the obstacle id and the state estimator id has to be equal!

    std::map<int, BaseStateEstimatorTaskSpace::UPtr> state_estimators_task_space; // map for all state estimators

    // Uncomment if specific state estimator for a dynamic obstacle is desired
    // State estimators Task Space
    // KalmanStateEstimatorTaskSpace::UPtr kalman1_ts_ptr = std::make_unique<KalmanStateEstimatorTaskSpace>(1);
    // KalmanStateEstimatorTaskSpace::UPtr kalman2_ts_ptr   = std::make_unique<KalmanStateEstimatorTaskSpace>(2);
    // PolynomStateEstimatorTaskSpace::UPtr polynom1_ts_ptr = std::make_unique<PolynomStateEstimatorTaskSpace>(1, 3, 7);  // for obstacle with id 1
    // PolynomStateEstimatorTaskSpace::UPtr polynom2_ts_ptr = std::make_unique<PolynomStateEstimatorTaskSpace>(2, 3, 7);  // for obstacle with id 2

    // Configure state estimators
    // configureKalmanFilterTaskSpace(*kalman1_ts_ptr, rate);
    // configureKalmanFilter(*kalman2_ts_ptr, rate);
    // configurePolynomFilterTaskSpace(*polynom1_ts_ptr, rate);
    // configurePolynomFilterTaskSpace(*polynom2_ts_ptr, rate);

    // state_estimators_task_space.insert(std::pair<int, BaseStateEstimatorTaskSpace::UPtr>(kalman1_ts_ptr->_id, std::move(kalman1_ts_ptr)));
    // state_estimators_task_space.insert(std::pair<int, BaseStateEstimatorTaskSpace::UPtr>(kalman2_ts_ptr->_id, std::move(kalman2_ts_ptr)));
    // state_estimators_task_space.insert(std::pair<int, BaseStateEstimatorTaskSpace::UPtr>(polynom1_ts_ptr->_id, std::move(polynom1_ts_ptr)));
    // state_estimators_task_space.insert(std::pair<int, BaseStateEstimatorTaskSpace::UPtr>(polynom2_ts_ptr->_id, std::move(polynom2_ts_ptr)));

    /***************************************************************************************************
        Define state estimators for humans
    /**************************************************************************************************/

    std::map<int, BaseStateEstimatorJointSpace::UPtr> state_estimators_joint_space;

    // foot print estimation (task space)
    if (foot_print_predicition_mode.compare("None") != 0)
    {
        if (foot_print_predicition_mode.compare("Polynom") == 0)
        {
            PolynomStateEstimatorTaskSpace::UPtr polynom_ts_foot_ptr =
                std::make_unique<PolynomStateEstimatorTaskSpace>(0, foot_polynom_fit_order, foot_polynom_fit_window); // human id=0
            configurePolynomFilterTaskSpace(*polynom_ts_foot_ptr, rate);
            state_estimators_task_space.insert(
                std::pair<int, BaseStateEstimatorTaskSpace::UPtr>(polynom_ts_foot_ptr->_id, std::move(polynom_ts_foot_ptr)));
        }
#ifdef SG
        else if (foot_print_predicition_mode.compare("Golay") == 0)
        {
            SavitzkyGolayEstimatorTaskSpace::UPtr golay_ts_foot_ptr = std::make_unique<SavitzkyGolayEstimatorTaskSpace>(
                0, foot_vel_filter_estimator_parameter, foot_acc_filter_estimator_parameter); // human id=0
            configureGolayFilterTaskSpace(*golay_ts_foot_ptr, rate);
            state_estimators_task_space.insert(
                std::pair<int, BaseStateEstimatorTaskSpace::UPtr>(golay_ts_foot_ptr->_id, std::move(golay_ts_foot_ptr)));
        }
#endif // SG
        else if (foot_print_predicition_mode.compare("Kalman") == 0)
        {
            KalmanStateEstimatorTaskSpace::UPtr kalman_ts_foot_ptr = std::make_unique<KalmanStateEstimatorTaskSpace>(0); // human id=0
            configureKalmanFilterTaskSpace(*kalman_ts_foot_ptr, rate);
            state_estimators_task_space.insert(
                std::pair<int, BaseStateEstimatorTaskSpace::UPtr>(kalman_ts_foot_ptr->_id, std::move(kalman_ts_foot_ptr)));
        }
    }

    // skeleton angle estimation (joint space)
    if (prediction_mode.compare("None") != 0)
    {
        if (prediction_mode.compare("Kalman") == 0)
        {
            KalmanStateEstimatorJointSpace::UPtr kalman1_js_ptr = std::make_unique<KalmanStateEstimatorJointSpace>(0); // human id=0
            configureKalmanFilterJointSpace(*kalman1_js_ptr, rate);
            state_estimators_joint_space.insert(std::pair<int, BaseStateEstimatorJointSpace::UPtr>(kalman1_js_ptr->_id, std::move(kalman1_js_ptr)));
        }
        else if (prediction_mode.compare("Polynom") == 0)
        {
            PolynomStateEstimatorJointSpace::UPtr polynom_js_ptr =
                std::make_unique<PolynomStateEstimatorJointSpace>(0, polynom_fit_order, polynom_fit_window); // human id=0
            configurePolynomFilterJointSpace(*polynom_js_ptr, rate);
            state_estimators_joint_space.insert(std::pair<int, BaseStateEstimatorJointSpace::UPtr>(polynom_js_ptr->_id, std::move(polynom_js_ptr)));
        }
#ifdef SG
        else if (prediction_mode.compare("Golay") == 0)
        {
            SavitzkyGolayEstimatorJointSpace::UPtr sg_js_ptr =
                std::make_unique<SavitzkyGolayEstimatorJointSpace>(0, vel_filter_estimator_parameter, acc_filter_estimator_parameter); // human id=0
            configureGolayFilterJointSpace(*sg_js_ptr, rate);
            state_estimators_joint_space.insert(std::pair<int, BaseStateEstimatorJointSpace::UPtr>(sg_js_ptr->_id, std::move(sg_js_ptr)));
        }
#endif // SG
    }

    int se_ts_size = state_estimators_task_space.size();
    int se_js_size = state_estimators_joint_space.size();
    // set state estimators for task space obstacles (obstacles + human foot prints) in the estimation process
    if (se_ts_size > 0)
        state_estimation_ptr->setEstimatorsTaskSpace(state_estimators_task_space);

    // set state estimators for joint space estimation in the estimation process
    if (se_js_size > 0)
        state_estimation_ptr->setEstimatorsJointSpace(state_estimators_joint_space);

    /***************************************************************************************************
        Define pipelines and processes that are executed iteratively
    /**************************************************************************************************/
    // Set workspace center to pedestal endpoint (this depends on the actual lab definiton)
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 1>(0, 3) << 0, 0, 0.8 + 0.1273;
    // Define workspace filter process
    workspace_filter_ptr->setWorkspacePose(pose);

    // Configure obstacle and human trajectories if automatic mode is used
    if (mode.compare("automatic") == 0)
    {
        ObstacleTrajectoryGenerator generator;
        automatic_simulator_ptr->setObstacleTrajectories(generator.parseObstacleTrajectories());
        automatic_simulator_ptr->setHumanTrajectories(generator.parseHumanTrajectories());
    }

    // Create and fill pipeline
    std::vector<BaseProcess::UPtr> pre_process_pipeline;
    pre_process_pipeline.push_back(std::move(load_obstacle_ptr));

    // Add obstacle mode process to pipeline that is required for the chosen workspace monitor mode
    if (mode.compare("tf") == 0)
    {
        pre_process_pipeline.push_back(std::move(tf_obstacle_ptr));
    }
    else if (mode.compare("manual") == 0)
    {
        pre_process_pipeline.push_back(std::move(manual_simulator_ptr));
    }
    else if (mode.compare("automatic") == 0)
    {
        pre_process_pipeline.push_back(std::move(automatic_simulator_ptr));
    }
    else if (mode.compare("bag") == 0)
    {
        pre_process_pipeline.push_back(std::move(tf_obstacle_ptr));
    }
    else if (mode.compare("mocap") == 0)
    {
        pre_process_pipeline.push_back(std::move(tf_obstacle_ptr));
    }
    else
    {
        ROS_ERROR("RobotWorkSpaceMonitor: Unknown obstacle mode.");
        return 1;
    }

    // Add estimation and extrapolation process if a task space or joint space estimator exists
    if (se_ts_size > 0 || se_js_size > 0)
    {
        pre_process_pipeline.push_back(std::move(state_estimation_ptr));
        pre_process_pipeline.push_back(std::move(state_extrapolation_ptr));
    }

    // Add RNN-SPL process for neural network human motion processes (estimation and extrapolation still required for foot print extrapolation)
    if (prediction_mode.compare("SPL") == 0)
        pre_process_pipeline.push_back(std::move(human_prediction_ptr));

    // Add uncertainty forecasting process
    if (uncertainty_mode.compare("None") != 0)
    {
        if (prediction_mode.compare("Polynom") == 0 || prediction_mode.compare("Golay") == 0 || prediction_mode.compare("Kalman") == 0)
        {
            ROS_INFO("Add uncertainty forecasting process");
            pre_process_pipeline.push_back(std::move(forecast_uncertainty_ptr));
        }
        else
        {
            ROS_WARN("RobotWorkSpaceMonitor: Uncertainty forecasting is not available without human motion extrapolation and is deactivated ");
        }
    }

    // Add motion classificator (is dynamic obstacle temporary static or not)
    pre_process_pipeline.push_back(std::move(motion_classification_ptr));

    // Define post process pipeline for filtering obstacles far away from origin
    std::vector<BaseProcess::UPtr> post_process_pipeline;
    post_process_pipeline.push_back(std::move(workspace_filter_ptr));

    // Create and load processors (processors execute the pipelines)
    ObstacleProcessor obstacle_pre_processor(pre_process_pipeline);
    ObstacleProcessor obstacle_post_processor(post_process_pipeline);

    // Initialize pipelines
    obstacle_pre_processor.initializePipeline();
    obstacle_post_processor.initializePipeline();

    /***************************************************************************************************
        Print information about the workspace monitor
    /**************************************************************************************************/
    ROS_INFO_STREAM("RobotWorkspaceMonitor: Obstacle mode: " << mode);
    ROS_INFO_STREAM("RobotWorkspaceMonitor: Prediction mode: " << prediction_mode);
    ROS_INFO_STREAM("RobotWorkspaceMonitor: Foot Print Prediction mode: " << foot_print_predicition_mode);
    ROS_INFO_STREAM("RobotWorkspaceMonitor: Uncertainty mode: " << uncertainty_mode);

    /***************************************************************************************************
        Start Main Loop
    /**************************************************************************************************/
    ROS_INFO("Workspace Monitor running...");
    while (ros::ok())
    {
        // Main proecessing obstacles
        obstacle_pre_processor.processPipeline();

        // Publish marker for all obstacles (also obstacles far away from origin)
        obstacle_publisher.publishMarker(obstacle_pre_processor._static_obstacles, obstacle_pre_processor._dynamic_obstacles,
                                         obstacle_pre_processor._humans, obstacle_pre_processor._utility_objects, obstacle_pre_processor._planes);

        obstacle_publisher.publishGazebo(obstacle_pre_processor._static_obstacles, obstacle_pre_processor._dynamic_obstacles,
                                         obstacle_pre_processor._humans, obstacle_pre_processor._utility_objects, obstacle_pre_processor._planes);

        // Postprocessing obstacles (filter obstacles far away)
        obstacle_post_processor.processPipeline(obstacle_pre_processor._static_obstacles, obstacle_pre_processor._dynamic_obstacles,
                                                obstacle_pre_processor._humans, obstacle_pre_processor._utility_objects,
                                                obstacle_pre_processor._planes);

        // Publish filtered obstacles and gazebo markers
        obstacle_publisher.publishObstacles(obstacle_post_processor._static_obstacles, obstacle_post_processor._dynamic_obstacles,
                                            obstacle_post_processor._humans, obstacle_post_processor._utility_objects,
                                            obstacle_post_processor._planes);

        ros::spinOnce(); // in case we have processes that have callbacks

        loop.sleep();
    }

    return 0;
}
