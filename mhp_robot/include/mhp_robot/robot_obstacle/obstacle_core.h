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

#ifndef OBSTACLE_CORE_H
#define OBSTACLE_CORE_H

// Base
#include <mhp_robot/robot_obstacle/obstacle_pipeline/base_process.h>

// Process
#include <mhp_robot/robot_obstacle/obstacle_pipeline/automatic_obstacle_simulator_process.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/forecast_uncertainty_process.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/human_motion_prediction_process.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/load_obstacle_process.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/manual_obstacle_simulator_process.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/motion_classification_process.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/state_estimation_process.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/state_extrapolation_process.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/tf_obstacle_process.h>
#include <mhp_robot/robot_obstacle/obstacle_pipeline/workspace_filter_process.h>

// State estimation
#include <mhp_robot/robot_obstacle/state_estimators/kalman_state_estimator_joint_space.h>
#include <mhp_robot/robot_obstacle/state_estimators/kalman_state_estimator_task_space.h>
#include <mhp_robot/robot_obstacle/state_estimators/polynom_state_estimator_joint_space.h>
#include <mhp_robot/robot_obstacle/state_estimators/polynom_state_estimator_task_space.h>
#include <mhp_robot/robot_obstacle/state_estimators/savitzky_golay_state_estimator_joint_space.h>
#include <mhp_robot/robot_obstacle/state_estimators/savitzky_golay_state_estimator_task_space.h>

// Misc
#include <mhp_robot/robot_obstacle/obstacle_processor.h>
#include <mhp_robot/robot_obstacle/obstacle_publisher.h>
#include <mhp_robot/robot_obstacle/obstacle_trajectory_generator.h>

#endif  // OBSTACLE_CORE_H
