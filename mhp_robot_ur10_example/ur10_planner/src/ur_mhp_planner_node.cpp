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

#include <mhp_planner/master/master.h>
#include <ros/ros.h>

// Factory includes
#include <mhp_planner/controllers/predictive_controller.h>
#include <mhp_planner/hypergraph/graph/hyper_graph_optimization_problem_vertex_based.h>
#include <mhp_planner/numerics/finite_differences.h>
#include <mhp_planner/numerics/finite_differences_collocation.h>
#include <mhp_planner/ocp/structured_ocp/discretization_grids/finite_differences_grid.h>
#include <mhp_planner/ocp/structured_ocp/structured_optimal_control_problem.h>
#include <mhp_planner/plants/plant_interface.h>
#include <mhp_planner/solvers/nlp_solver_ipopt.h>
#include <mhp_planner/systems/standard_filters.h>
#include <mhp_planner/systems/system_dynamics_interface.h>

#include <ur10_planner/ocp/constraints/ur_acceleration_constraint_corbo.h>
#include <ur10_planner/ocp/constraints/ur_collision_constraint_corbo.h>
#include <ur10_planner/ocp/costs/ur_quadratic_cost_joint_space_corbo.h>
#include <ur10_planner/ocp/potentials/ur_collision_potential_mohri_corbo.h>
#include <ur10_planner/ocp/gains/ur_information_gain_simple_corbo.h>
#include <ur10_planner/ocp/ur_final_stage_cost_corbo.h>
#include <ur10_planner/ocp/ur_final_state_constraint_joint_space_corbo.h>
#include <ur10_planner/ocp/ur_inequality_constraint_corbo.h>
#include <ur10_planner/ocp/ur_stage_cost_corbo.h>
#include <ur10_planner/plants/ur_robot_corbo.h>
#include <ur10_planner/tasks/ur_task_corbo.h>

int main(int argc, char** argv)
{
#ifndef NDEBUG
    sleep(5);
#endif
    ros::init(argc, argv, "mhp_planner_node");

    ros::NodeHandle n("~");

    mhp_planner::Master master;

    master.loadFromParameterServer("ur_mhp_planner");

    master.start();

    ROS_INFO("Planner started");

    ros::spin();

    return 0;
}
