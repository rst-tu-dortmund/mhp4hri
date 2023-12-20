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

#ifndef COMMON_H
#define COMMON_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <mhp_robot/MsgBoundingBox.h>
#include <mhp_robot/MsgHuman.h>
#include <mhp_robot/MsgObstacle.h>
#include <mhp_robot/MsgObstacleList.h>
#include <mhp_robot/MsgPlane.h>
#include <mhp_robot/MsgUtilityObject.h>
#include <mhp_robot/robot_misc/bounding_box.h>
#include <mhp_robot/robot_misc/human.h>
#include <mhp_robot/robot_misc/obstacle.h>
#include <mhp_robot/robot_misc/plane.h>
#include <mhp_robot/robot_misc/utility_object.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <Eigen/Dense>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <string>

namespace mhp_robot {
namespace robot_misc {

class Common
{
 public:
    static Eigen::Matrix<double, 4, 4> rotx(double angle);
    static Eigen::Matrix<double, 4, 4> roty(double angle);
    static Eigen::Matrix<double, 4, 4> rotz(double angle);

    static void rotx(double angle, Eigen::Ref<Eigen::Matrix<double, 4, 4>> R);
    static void roty(double angle, Eigen::Ref<Eigen::Matrix<double, 4, 4>> R);
    static void rotz(double angle, Eigen::Ref<Eigen::Matrix<double, 4, 4>> R);

    static void rotxyz(double rotx, double roty, double rotz, Eigen::Ref<Eigen::Matrix<double, 3, 3>> R);

    static Eigen::Matrix<double, 4, 4> poseFromPlane(const Eigen::Ref<const Eigen::Vector3d>& q, const Eigen::Ref<const Eigen::Vector3d>& n);

    static void interpolatePose(Eigen::Ref<Eigen::Matrix4d> pose, const Eigen::Ref<const Eigen::Matrix4d>& pose1,
                                const Eigen::Ref<const Eigen::Matrix4d>& pose2, double scaling);

    static void interpolateVector(Eigen::Ref<Eigen::Vector3d> vector, const Eigen::Ref<const Eigen::Vector3d>& vector1,
                                  const Eigen::Ref<const Eigen::Vector3d>& vector2, double scaling);

    static void poseEigenToMsg(const Eigen::Ref<const Eigen::Matrix4d>& pose_eigen, geometry_msgs::Pose& pose_msg);
    static void poseEigenToMsg(const Eigen::Ref<const Eigen::Matrix4d>& pose_eigen, geometry_msgs::Transform& transform_msg);
    static void poseMsgToEigen(Eigen::Ref<Eigen::Matrix4d> pose_eigen, const geometry_msgs::Pose& pose_msg);
    static void poseMsgToEigen(Eigen::Ref<Eigen::Matrix4d> pose_eigen, const geometry_msgs::Transform& transform_msg);

    static void vectorEigenToMsg(const Eigen::Ref<const Eigen::Vector3d>& vector_eigen, geometry_msgs::Vector3& vector_msg);
    static void vectorMsgToEigen(Eigen::Ref<Eigen::Vector3d> vector_eigen, const geometry_msgs::Vector3& vector_msg);

    static void generateHumanMsg(const Human& human, MsgHuman& human_msg);
    static void generateUtilityObjectMsg(const UtilityObject& utility_object, MsgUtilityObject& utility_object_msg);
    static void generatePlaneMsg(const Plane& plane, MsgPlane& plane_msg);
    static void generateObstacleMsg(const Obstacle& obstacle, MsgObstacle& obstacle_msg);
    static void generateBoundingBoxMsg(const BoundingBox& bounding_box, MsgBoundingBox& bounding_box_msg);
    static void generateStateMsg(const State& state, trajectory_msgs::MultiDOFJointTrajectory& states_msg);
    static void generateUncertaintyStateMsg(const std::vector<State>& uncertainty_states, std::vector<MsgUncertaintyState>& uncertainty_states_msg);

    static void generateHuman(Human& human, const MsgHuman& human_msg);
    static void generateUtilityObject(UtilityObject& utility_object, const MsgUtilityObject& utility_object_msg);
    static void generatePlane(Plane& plane, const MsgPlane& plane_msg);
    static void generateBoundingBox(BoundingBox& bounding_box, const MsgBoundingBox& bounding_box_msg);
    static void generateObstacle(Obstacle& obstacle, const MsgObstacle& obstacle_msg);
    static void generateState(State& state, const trajectory_msgs::MultiDOFJointTrajectory& states_msg);
    static void generateUncertaintyState(std::vector<State>& state, const std::vector<MsgUncertaintyState>& states_msg);

    static void parseObstacleMsg(const MsgObstacleListConstPtr& obstacle_list_msg, std::vector<UtilityObject>& utility_objects);

    static void parseObstacleMsg(const MsgObstacleListConstPtr& obstacle_list_msg, std::vector<Obstacle>& static_obstacles,
                                 std::vector<Obstacle>& dynamic_obstacles);

    static void parseObstacleMsg(const MsgObstacleListConstPtr& obstacle_list_msg, std::vector<Obstacle>& static_obstacles,
                                 std::vector<Obstacle>& dynamic_obstacles, std::vector<Human>& humans, std::vector<Plane>& planes);

    static void parseObstacleMsg(const MsgObstacleListConstPtr& obstacle_list_msg, std::vector<Obstacle>& static_obstacles,
                                 std::vector<Obstacle>& dynamic_obstacles, std::vector<Human>& humans, std::vector<UtilityObject>& utility_objects,
                                 std::vector<Plane>& planes);

    static void setMarkerPose(visualization_msgs::InteractiveMarker& int_marker, const Obstacle& obstacle);

    static void setMarkerPose(visualization_msgs::InteractiveMarker& int_marker, const Plane& plane);

    static double matrixConditionNum(const Eigen::Ref<const Eigen::MatrixXd>& matrix);

    static Eigen::Matrix4d getTransformation(const std::string& base_frame, const std::string& target_frame, tf::TransformListener& tf_listener);

    static std::pair<double, double> calculatePoseError(const Eigen::Ref<const Eigen::Matrix4d>& pose1,
                                                        const Eigen::Ref<const Eigen::Matrix4d>& pose2);
    static double calculateJointError(const std::vector<double>& joint_position1, const std::vector<double>& joint_position2);

    static void createControlMarker(visualization_msgs::InteractiveMarkerControl& control, visualization_msgs::InteractiveMarker& int_marker);
    static void createControlMarkerReduced(visualization_msgs::InteractiveMarkerControl& control, visualization_msgs::InteractiveMarker& int_marker);

    template <typename T>
    static int sgn(const T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    // Source: https://stackoverflow.com/a/10848293
    template <typename T>
    static T normalPDF(T x, T m, T s)
    {
        static const T inv_sqrt_2pi = 0.3989422804014327;
        T a                         = (x - m) / s;

        return inv_sqrt_2pi / s * std::exp(-T(0.5) * a * a);
    }

    static boost::property_tree::ptree readJSON(const std::string& json_string);
    static std::string writeJSON(const boost::property_tree::ptree& tree, bool fix_type = false, bool pretty = false);

    template <class Type>
    static std::vector<Type> parseJSONList(const boost::property_tree::ptree& tree)
    {
        std::vector<Type> list;
        for (const boost::property_tree::ptree::value_type& data : tree) list.push_back(data.second.get_value<Type>());
        return list;
    }

    static bool waitFor(
        std::function<bool()> test, double timeout = 1.0, const std::string& timeout_msg = "Timeout expired.", double rate = 100,
        std::function<void()> body = []() {});

    static double wrapPItoPI(double angle);

    static void writeToCSV(const std::string& name, const Eigen::Ref<const Eigen::MatrixXd>& matrix);

    static bool writeToFile(const char* const text, const std::string& filename);
};
}  // namespace robot_misc
}  // namespace mhp_robot

#endif  // COMMON_H
