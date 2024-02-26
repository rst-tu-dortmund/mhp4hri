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

#include <mhp_robot/robot_misc/common.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_eigen.h>
#include <regex>

namespace mhp_robot {
namespace robot_misc {

Eigen::Matrix<double, 4, 4> Common::rotx(double angle)
{
    Eigen::Matrix<double, 4, 4> R;
    R << 1, 0, 0, 0, 0, cos(angle), -sin(angle), 0, 0, sin(angle), cos(angle), 0, 0, 0, 0, 1;
    return R;
}

Eigen::Matrix<double, 4, 4> Common::roty(double angle)
{
    Eigen::Matrix<double, 4, 4> R;
    R << cos(angle), 0, sin(angle), 0, 0, 1, 0, 0, -sin(angle), 0, cos(angle), 0, 0, 0, 0, 1;
    return R;
}

Eigen::Matrix<double, 4, 4> Common::rotz(double angle)
{
    Eigen::Matrix<double, 4, 4> R;
    R << cos(angle), -sin(angle), 0, 0, sin(angle), cos(angle), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    return R;
}

Eigen::Matrix<double, 4, 4> Common::poseFromPlane(const Eigen::Ref<const Eigen::Vector3d>& q, const Eigen::Ref<const Eigen::Vector3d>& n)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    T.block<3, 1>(0, 3) = q;

    double x = n(0);
    double y = n(1);
    double a = 1.0;
    double b = 0.0;

    if (std::abs(x) < 1e-4)
    {
        a = 1.0;
        b = 0.0;
    }
    else if (std::abs(y) < 1e-4)
    {
        a = 0.0;
        b = 1.0;
    }
    else
    {
        a = sqrt(y * y / (x * x + y * y));
        b = -a * x / y;
    }

    Eigen::Vector3d v, w;
    v << a, b, 0;
    w = n.cross(v);

    T.block<3, 1>(0, 0) = v;
    T.block<3, 1>(0, 1) = w;
    T.block<3, 1>(0, 2) = n;

    return T;
}

void Common::rotx(double angle, Eigen::Ref<Eigen::Matrix<double, 4, 4>> R)
{
    R << 1, 0, 0, 0, 0, cos(angle), -sin(angle), 0, 0, sin(angle), cos(angle), 0, 0, 0, 0, 1;
}

void Common::roty(double angle, Eigen::Ref<Eigen::Matrix<double, 4, 4>> R)
{
    R << cos(angle), 0, sin(angle), 0, 0, 1, 0, 0, -sin(angle), 0, cos(angle), 0, 0, 0, 0, 1;
}

void Common::rotz(double angle, Eigen::Ref<Eigen::Matrix<double, 4, 4>> R)
{
    R << cos(angle), -sin(angle), 0, 0, sin(angle), cos(angle), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
}

void Common::rotxyz(double rotx, double roty, double rotz, Eigen::Ref<Eigen::Matrix<double, 3, 3>> R)
{
    double c1 = cos(rotx);
    double c2 = cos(roty);
    double c3 = cos(rotz);

    double s1 = sin(rotx);
    double s2 = sin(roty);
    double s3 = sin(rotz);

    R << c2 * c3, -c2 * s3, s2, s1 * s2 * c3 + c1 * s3, -s1 * s2 * s3 + c1 * c3, -s1 * c2, -c1 * s2 * c3 + s1 * s3, c1 * s2 * s3 + s1 * c3, c1 * c2;
}

void Common::interpolatePose(Eigen::Ref<Eigen::Matrix4d> pose, const Eigen::Ref<const Eigen::Matrix4d>& pose1,
                             const Eigen::Ref<const Eigen::Matrix4d>& pose2, double scaling)
{
    Eigen::Quaterniond q1(pose1.block<3, 3>(0, 0));
    Eigen::Quaterniond q2(pose2.block<3, 3>(0, 0));

    Eigen::Quaterniond q   = q1.slerp(scaling, q2);
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose.block<3, 1>(0, 3) = pose1.block<3, 1>(0, 3) + (pose2.block<3, 1>(0, 3) - pose1.block<3, 1>(0, 3)) * scaling;
}

void Common::interpolateVector(Eigen::Ref<Eigen::Vector3d> vector, const Eigen::Ref<const Eigen::Vector3d>& vector1,
                               const Eigen::Ref<const Eigen::Vector3d>& vector2, double scaling)
{
    vector = vector + (vector2 - vector1) * scaling;
    // TODO(renz) Check if correct not sure with first vector
}

void Common::poseEigenToMsg(const Eigen::Ref<const Eigen::Matrix4d>& pose_eigen, geometry_msgs::Pose& pose_msg)
{
    Eigen::Quaterniond quat(pose_eigen.block<3, 3>(0, 0));
    quat.normalize();

    pose_msg.position.x = pose_eigen(0, 3);
    pose_msg.position.y = pose_eigen(1, 3);
    pose_msg.position.z = pose_eigen(2, 3);

    pose_msg.orientation.w = quat.w();
    pose_msg.orientation.x = quat.x();
    pose_msg.orientation.y = quat.y();
    pose_msg.orientation.z = quat.z();
}

void Common::poseEigenToMsg(const Eigen::Ref<const Eigen::Matrix4d>& pose_eigen, geometry_msgs::Transform& transform_msg)
{
    Eigen::Quaterniond quat(pose_eigen.block<3, 3>(0, 0));
    quat.normalize();

    transform_msg.translation.x = pose_eigen(0, 3);
    transform_msg.translation.y = pose_eigen(1, 3);
    transform_msg.translation.z = pose_eigen(2, 3);

    transform_msg.rotation.w = quat.w();
    transform_msg.rotation.x = quat.x();
    transform_msg.rotation.y = quat.y();
    transform_msg.rotation.z = quat.z();
}

void Common::poseMsgToEigen(Eigen::Ref<Eigen::Matrix4d> pose_eigen, const geometry_msgs::Pose& pose_msg)
{
    Eigen::Quaterniond quat(pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z);
    quat.normalize();

    pose_eigen.block<3, 3>(0, 0) = quat.toRotationMatrix();
    pose_eigen(0, 3)             = pose_msg.position.x;
    pose_eigen(1, 3)             = pose_msg.position.y;
    pose_eigen(2, 3)             = pose_msg.position.z;
}

void Common::poseMsgToEigen(Eigen::Ref<Eigen::Matrix4d> pose_eigen, const geometry_msgs::Transform& transform_msg)
{
    Eigen::Quaterniond quat(transform_msg.rotation.w, transform_msg.rotation.x, transform_msg.rotation.y, transform_msg.rotation.z);
    quat.normalize();

    pose_eigen.block<3, 3>(0, 0) = quat.toRotationMatrix();
    pose_eigen(0, 3)             = transform_msg.translation.x;
    pose_eigen(1, 3)             = transform_msg.translation.y;
    pose_eigen(2, 3)             = transform_msg.translation.z;
}

void Common::vectorEigenToMsg(const Eigen::Ref<const Eigen::Vector3d>& vector_eigen, geometry_msgs::Vector3& vector_msg)
{
    vector_msg.x = vector_eigen(0);
    vector_msg.y = vector_eigen(1);
    vector_msg.z = vector_eigen(2);
}

void Common::vectorMsgToEigen(Eigen::Ref<Eigen::Vector3d> vector_eigen, const geometry_msgs::Vector3& vector_msg)
{
    vector_eigen << vector_msg.x, vector_msg.y, vector_msg.z;
}

void Common::generateHumanMsg(const Human& human, MsgHuman& human_msg)
{
    human_msg.id                 = human._id;
    human_msg.name               = human._name;
    human_msg.uncertainty_mode   = human._uncertainty_mode;
    human_msg.skeleton_splitting = human._skeleton_splitting;
    human_msg.ms_mapping = std::vector<int>(human._ms_mapping.data(), human._ms_mapping.data() + human._ms_mapping.rows() * human._ms_mapping.cols());

    for (auto& part : human._body_parts)
    {
        MsgObstacle body_part_msg;
        generateObstacleMsg(part.second, body_part_msg);
        human_msg.body_parts.push_back(body_part_msg);
    }
}

void Common::generateUtilityObjectMsg(const UtilityObject& utility_object, MsgUtilityObject& utility_object_msg)
{
    generateObstacleMsg(utility_object, utility_object_msg.object);
    utility_object_msg.held = utility_object.held;
}

void Common::generatePlaneMsg(const Plane& plane, MsgPlane& plane_msg)
{
    plane_msg.id             = plane.id;
    plane_msg.name           = plane.name;
    plane_msg.group_id       = plane.group_id;
    plane_msg.support_vector = std::vector<double>(plane.q.data(), plane.q.data() + 3);
    plane_msg.normal_vector  = std::vector<double>(plane.n.data(), plane.n.data() + 3);
}

void Common::generateObstacleMsg(const Obstacle& obstacle, MsgObstacle& obstacle_msg)
{
    generateBoundingBoxMsg(obstacle.bounding_box, obstacle_msg.bounding_box);
    generateStateMsg(obstacle.state, obstacle_msg.states);
    generateUncertaintyStateMsg(obstacle.uncertainty_states, obstacle_msg.uncertainty_states);
    obstacle_msg.id               = obstacle.id;
    obstacle_msg.name             = obstacle.name;
    obstacle_msg.group_id         = obstacle.group_id;
    obstacle_msg.temporary_static = obstacle.temporary_static;
}

void Common::generateBoundingBoxMsg(const BoundingBox& bounding_box, MsgBoundingBox& bounding_box_msg)
{
    bounding_box_msg.active        = bounding_box.active;
    bounding_box_msg.extruded      = bounding_box.extruded;
    bounding_box_msg.type          = (int)bounding_box.type;
    bounding_box_msg.length_x      = bounding_box.length_x;
    bounding_box_msg.length_y      = bounding_box.length_y;
    bounding_box_msg.radius        = bounding_box.radius;
    bounding_box_msg.future_radius = bounding_box.future_radius;
    poseEigenToMsg(bounding_box.T, bounding_box_msg.pose);
}

void Common::generateStateMsg(const State& state, trajectory_msgs::MultiDOFJointTrajectory& states_msg)
{
    states_msg.points.clear();

    for (int i = 0; i < (int)state._times.size(); ++i)
    {
        // create state
        states_msg.points.emplace_back();

        // create empty transform
        states_msg.points.back().transforms.emplace_back();

        // create empty velocity
        states_msg.points.back().velocities.emplace_back();

        // create empty accelerations
        states_msg.points.back().accelerations.emplace_back();

        // Fill pose
        poseEigenToMsg(state._poses[i], states_msg.points[i].transforms[0]);

        // Fill twist
        vectorEigenToMsg(state._angular_velocities[i], states_msg.points[i].velocities[0].angular);
        vectorEigenToMsg(state._linear_velocities[i], states_msg.points[i].velocities[0].linear);

        // Fill acceleration
        vectorEigenToMsg(state._angular_accelerations[i], states_msg.points[i].accelerations[0].angular);
        vectorEigenToMsg(state._linear_accelerations[i], states_msg.points[i].accelerations[0].linear);

        // Fill time
        states_msg.points[i].time_from_start = ros::Duration(state._times[i]);
    }
}

void Common::generateUncertaintyStateMsg(const std::vector<State>& uncertainty_states, std::vector<MsgUncertaintyState>& uncertainty_states_msg)
{
    uncertainty_states_msg.clear();
    for (int cnt = 0; cnt < uncertainty_states.size(); ++cnt)
    {
        // create empty state msg
        uncertainty_states_msg.emplace_back();

        // fill up state msg
        generateStateMsg(uncertainty_states.at(cnt), uncertainty_states_msg.at(cnt).uncertainty_state);
        uncertainty_states_msg.at(cnt).uncertainty_instance = uncertainty_states.at(cnt).getUncertaintyInstance();
    }
}

void Common::generateHuman(Human& human, const MsgHuman& human_msg)
{
    human._id                 = human_msg.id;
    human._name               = human_msg.name;
    human._skeleton_splitting = human_msg.skeleton_splitting;
    human._uncertainty_mode   = human_msg.uncertainty_mode;
    human._ms_mapping         = Eigen::VectorXi::Map(&human_msg.ms_mapping[0], human_msg.ms_mapping.size(), 1);

    for (int i = 0; i < human_msg.body_parts.size(); ++i)
    {
        Obstacle obs;
        generateObstacle(obs, human_msg.body_parts[i]);
        human._body_parts[obs.name] = obs;
    }
}

void Common::generateUtilityObject(UtilityObject& utility_object, const MsgUtilityObject& utility_object_msg)
{
    generateObstacle(utility_object, utility_object_msg.object);
    utility_object.held = utility_object_msg.held;
}

void Common::generatePlane(Plane& plane, const MsgPlane& plane_msg)
{
    plane.id       = plane_msg.id;
    plane.name     = plane_msg.name;
    plane.group_id = plane_msg.group_id;
    plane.q        = Eigen::Map<const Eigen::Vector3d>(plane_msg.support_vector.data());
    plane.n        = Eigen::Map<const Eigen::Vector3d>(plane_msg.normal_vector.data());
}

void Common::generateBoundingBox(BoundingBox& bounding_box, const MsgBoundingBox& bounding_box_msg)
{
    bounding_box.active        = bounding_box_msg.active;
    bounding_box.extruded      = bounding_box_msg.extruded;
    bounding_box.length_x      = bounding_box_msg.length_x;
    bounding_box.length_y      = bounding_box_msg.length_y;
    bounding_box.radius        = bounding_box_msg.radius;
    bounding_box.future_radius = bounding_box_msg.future_radius;
    bounding_box.type          = static_cast<BoundingBoxType>(bounding_box_msg.type);
    poseMsgToEigen(bounding_box.T, bounding_box_msg.pose);
}

void Common::generateObstacle(Obstacle& obstacle, const MsgObstacle& obstacle_msg)
{
    obstacle.id               = obstacle_msg.id;
    obstacle.name             = obstacle_msg.name;
    obstacle.group_id         = obstacle_msg.group_id;
    obstacle.temporary_static = obstacle_msg.temporary_static;
    generateBoundingBox(obstacle.bounding_box, obstacle_msg.bounding_box);
    generateState(obstacle.state, obstacle_msg.states);
    generateUncertaintyState(obstacle.uncertainty_states, obstacle_msg.uncertainty_states);
}

void Common::generateState(State& state, const trajectory_msgs::MultiDOFJointTrajectory& states_msg)
{
    state.clear();

    for (int i = 0; i < (int)states_msg.points.size(); ++i)
    {
        // Create pose
        state._poses.emplace_back();

        // Create twist
        state._angular_velocities.emplace_back();
        state._linear_velocities.emplace_back();

        // Create acceleration
        state._angular_accelerations.emplace_back();
        state._linear_accelerations.emplace_back();

        // Create time
        state._times.emplace_back();

        // Fill pose
        poseMsgToEigen(state._poses[i], states_msg.points[i].transforms[0]);

        // Fill twist
        vectorMsgToEigen(state._angular_velocities[i], states_msg.points[i].velocities[0].angular);
        vectorMsgToEigen(state._linear_velocities[i], states_msg.points[i].velocities[0].linear);

        // Fill accelerations
        vectorMsgToEigen(state._angular_accelerations[i], states_msg.points[i].accelerations[0].angular);
        vectorMsgToEigen(state._linear_accelerations[i], states_msg.points[i].accelerations[0].linear);

        // Fill time
        state._times[i] = states_msg.points[i].time_from_start.toSec();
    }
}

void Common::generateUncertaintyState(std::vector<State>& uncertainty_states, const std::vector<MsgUncertaintyState>& uncertainty_states_msg)
{
    uncertainty_states.clear();
    for (int cnt = 0; cnt < uncertainty_states_msg.size(); ++cnt)
    {
        // Create uncertainty state element
        uncertainty_states.emplace_back();

        // Fill uncertainty state element
        generateState(uncertainty_states.at(cnt), uncertainty_states_msg.at(cnt).uncertainty_state);
        uncertainty_states.at(cnt)._uncertainty_instance = uncertainty_states_msg.at(cnt).uncertainty_instance;
    }
}

void Common::parseObstacleMsg(const MsgObstacleListConstPtr& obstacle_list_msg, std::vector<UtilityObject>& utility_objects)
{
    std::vector<Human> humans;
    std::vector<Plane> planes;
    std::vector<Obstacle> dynamics, statics;
    parseObstacleMsg(obstacle_list_msg, statics, dynamics, humans, utility_objects, planes);
}

void Common::parseObstacleMsg(const MsgObstacleListConstPtr& obstacle_list_msg, std::vector<Obstacle>& static_obstacles,
                              std::vector<Obstacle>& dynamic_obstacles)
{
    std::vector<UtilityObject> util;
    std::vector<Human> humans;
    std::vector<Plane> planes;
    parseObstacleMsg(obstacle_list_msg, static_obstacles, dynamic_obstacles, humans, util, planes);
}

void Common::parseObstacleMsg(const MsgObstacleListConstPtr& obstacle_list_msg, std::vector<Obstacle>& static_obstacles,
                              std::vector<Obstacle>& dynamic_obstacles, std::vector<Human>& humans, std::vector<Plane>& planes)
{
    std::vector<UtilityObject> util;
    parseObstacleMsg(obstacle_list_msg, static_obstacles, dynamic_obstacles, humans, util, planes);
}

void Common::parseObstacleMsg(const MsgObstacleListConstPtr& obstacle_list_msg, std::vector<Obstacle>& static_obstacles,
                              std::vector<Obstacle>& dynamic_obstacles, std::vector<Human>& humans, std::vector<UtilityObject>& utility_objects,
                              std::vector<Plane>& planes)
{
    static_obstacles.clear();
    dynamic_obstacles.clear();
    humans.clear();
    planes.clear();
    utility_objects.clear();

    Obstacle obstacle;
    for (int i = 0; i < (int)obstacle_list_msg->static_obstacles.size(); ++i)
    {
        generateObstacle(obstacle, obstacle_list_msg->static_obstacles[i]);
        static_obstacles.push_back(obstacle);
    }

    for (int i = 0; i < (int)obstacle_list_msg->dynamic_obstacles.size(); ++i)
    {
        generateObstacle(obstacle, obstacle_list_msg->dynamic_obstacles[i]);
        dynamic_obstacles.push_back(obstacle);
    }

    Human human(0);
    for (int i = 0; i < (int)obstacle_list_msg->humans.size(); ++i)
    {
        generateHuman(human, obstacle_list_msg->humans[i]);
        humans.push_back(human);
    }

    UtilityObject object;
    for (int i = 0; i < (int)obstacle_list_msg->utility_objects.size(); ++i)
    {
        generateUtilityObject(object, obstacle_list_msg->utility_objects[i]);
        utility_objects.push_back(object);
    }

    Plane plane;
    for (int i = 0; i < (int)obstacle_list_msg->planes.size(); ++i)
    {
        generatePlane(plane, obstacle_list_msg->planes[i]);
        planes.push_back(plane);
    }
}

void Common::setMarkerPose(visualization_msgs::InteractiveMarker& int_marker, const Obstacle& obstacle)
{
    geometry_msgs::Pose pose_msg;
    robot_misc::Common::poseEigenToMsg(obstacle.state.getPose(), pose_msg);

    int_marker.pose.orientation = pose_msg.orientation;
    int_marker.pose.position    = pose_msg.position;
}

void Common::setMarkerPose(visualization_msgs::InteractiveMarker& int_marker, const Plane& plane)
{
    Eigen::Quaterniond orientation = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitZ(), plane.n);
    orientation.normalize();

    int_marker.pose.orientation.x = orientation.x();
    int_marker.pose.orientation.y = orientation.y();
    int_marker.pose.orientation.z = orientation.z();
    int_marker.pose.orientation.w = orientation.w();

    int_marker.pose.position.x = plane.q(0);
    int_marker.pose.position.y = plane.q(1);
    int_marker.pose.position.z = plane.q(2);
}

double Common::matrixConditionNum(const Eigen::Ref<const Eigen::MatrixXd>& matrix)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd s = svd.singularValues();

    return (s(0) / s.tail<1>()(0));
}

Eigen::Matrix4d Common::getTransformation(const std::string& base_frame, const std::string& target_frame, tf::TransformListener& tf_listener)
{
    tf::StampedTransform stamped_transform;
    tf::Transform transform;

    Eigen::Affine3d ee;

    tf_listener.waitForTransform(base_frame, target_frame, ros::Time(0), ros::Duration(10.0));
    tf_listener.lookupTransform(base_frame, target_frame, ros::Time(0), stamped_transform);
    transform = stamped_transform;
    tf::transformTFToEigen(transform, ee);

    return ee.matrix();
}

std::pair<double, double> Common::calculatePoseError(const Eigen::Ref<const Eigen::Matrix4d>& pose1, const Eigen::Ref<const Eigen::Matrix4d>& pose2)
{
    std::pair<double, double> error;
    error.first  = 0;
    error.second = 0;

    // position error
    error.first += (pose2(0, 3) - pose1(0, 3)) * (pose2(0, 3) - pose1(0, 3));
    error.first += (pose2(1, 3) - pose1(1, 3)) * (pose2(1, 3) - pose1(1, 3));
    error.first += (pose2(2, 3) - pose1(2, 3)) * (pose2(2, 3) - pose1(2, 3));

    // Orientation error (quaternion)
    Eigen::Quaterniond q1(pose1.block<3, 3>(0, 0));
    Eigen::Quaterniond q2(pose2.block<3, 3>(0, 0));

    tf::Quaternion q_t(q1.x(), q1.y(), q1.z(), q1.w());
    tf::Quaternion q_m(q2.x(), q2.y(), q2.z(), q2.w());

    double error_o = tf::angleShortestPath(q_m, q_t);
    error.second   = error_o * error_o;

    return error;
}

double Common::calculateJointError(const std::vector<double>& joint_position1, const std::vector<double>& joint_position2)
{
    double error = 0;

    for (int i = 0; i < joint_position1.size() && i < joint_position2.size(); ++i)
    {
        error += (joint_position1[i] - joint_position2[i]) * (joint_position1[i] - joint_position2[i]);
    }
    return sqrt(error);
}

void Common::createControlMarker(visualization_msgs::InteractiveMarkerControl& control, visualization_msgs::InteractiveMarker& int_marker)
{
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

    control.orientation      = tf2::toMsg(tf2::Quaternion(1, 0, 0, 1).normalize());
    control.name             = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name             = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation      = tf2::toMsg(tf2::Quaternion(0, 1, 0, 1).normalize());
    control.name             = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name             = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation      = tf2::toMsg(tf2::Quaternion(0, 0, 1, 1).normalize());
    control.name             = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name             = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
}

void Common::createControlMarkerReduced(visualization_msgs::InteractiveMarkerControl& control, visualization_msgs::InteractiveMarker& int_marker)
{
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

    control.orientation      = tf2::toMsg(tf2::Quaternion(1, 0, 0, 1).normalize());
    control.name             = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation      = tf2::toMsg(tf2::Quaternion(0, 1, 0, 1).normalize());
    control.name             = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name             = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation      = tf2::toMsg(tf2::Quaternion(0, 0, 1, 1).normalize());
    control.name             = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
}

boost::property_tree::ptree Common::readJSON(const std::string& json_string)
{
    boost::property_tree::ptree tree;

    if (json_string != "")
    {
        std::stringstream stream;
        stream << json_string;
        boost::property_tree::read_json(stream, tree);
    }

    return tree;
}

std::string Common::writeJSON(const boost::property_tree::ptree& tree, bool fix_type, bool pretty)
{
    std::stringstream stream;
    boost::property_tree::write_json(stream, tree, pretty);
    std::string json_string = stream.str();

    /*
     * Currently, boost::property_tree does not preserve datatypes,
     * therefore all data are being represented as strings.
     * The following block will find float and boolean literals and
     * remove the surrounding quotes.
     *
     * It is now by default disabled since the Sawyer robot can still
     * parse the strings correctly.
     */
    if (fix_type)
    {
        // Remove quotes around numbers.
        std::regex reg_float("\\\"(-?(?:0|[1-9]\\d*)(?:\\.\\d+)?(?:[eE][+-]?\\d+)?)\\\"");
        json_string = std::regex_replace(json_string, reg_float, "$1");

        // Remove quotes around "true" keyword.
        std::regex reg_true("\\\"(true)\\\"");
        json_string = std::regex_replace(json_string, reg_true, "$1");

        // Remove quotes around "false" keyword.
        std::regex reg_false("\\\"(false)\\\"");
        json_string = std::regex_replace(json_string, reg_false, "$1");
    }

    // Remove linebreak at the end.
    json_string.erase(json_string.length() - 1);

    return json_string;
}

bool Common::waitFor(std::function<bool()> test, double timeout, const std::string& timeout_msg, double rate, std::function<void()> body)
{
    if (timeout < 0.0) timeout = std::numeric_limits<double>::infinity();
    double end_time = ros::Time::now().toSec() + timeout;
    ros::Rate loop_rate(rate);
    while (!test())
    {
        if (ros::Time::now().toSec() >= end_time)
        {
            if (!timeout_msg.empty()) ROS_WARN_STREAM(timeout_msg);
            return false;
        }
        body();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return true;
}

double Common::wrapPItoPI(double angle)
{
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    return angle - M_PI;
}

void Common::writeToCSV(const std::string& name, const Eigen::Ref<const Eigen::MatrixXd>& matrix)
{
    // Source: https://stackoverflow.com/a/23566993

    std::ofstream file(name.c_str());
    file << matrix.format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n"));
    file.close();
}

bool Common::writeToFile(const char* const text, const std::string& filename)
{
    if (filename.empty())
    {
        return false;
    }

    std::ofstream file;
    file.open(filename);

    if (file.is_open())
    {
        file << text;
        file.close();

        return true;
    }
    else
    {
        return false;
    }
}

}  // namespace robot_misc
}  // namespace mhp_robot
