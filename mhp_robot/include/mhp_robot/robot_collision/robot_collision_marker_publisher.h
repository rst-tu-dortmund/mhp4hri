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

#ifndef ROBOT_COLLISION_MARKER_PUBLISHER_H
#define ROBOT_COLLISION_MARKER_PUBLISHER_H

#include <mhp_robot/MsgObstacleList.h>
#include <mhp_robot/robot_collision/robot_collision.h>
#include <mhp_robot/robot_misc/color.h>
#include <mhp_robot/robot_obstacle/obstacle_list.h>
#include <sensor_msgs/JointState.h>
#include <string>

namespace mhp_robot {
namespace robot_collision {

class RobotCollisionMarkerPublisher
{
 public:
    RobotCollisionMarkerPublisher(const std::string& joint_state_topic, const std::string& obstacle_topic, RobotCollision::UPtr robot_collision,
                                  robot_misc::RobotUtility::UPtr robot_utility);

    RobotCollisionMarkerPublisher(const RobotCollisionMarkerPublisher&)            = delete;
    RobotCollisionMarkerPublisher(RobotCollisionMarkerPublisher&&)                 = delete;
    RobotCollisionMarkerPublisher& operator=(const RobotCollisionMarkerPublisher&) = delete;
    RobotCollisionMarkerPublisher& operator=(RobotCollisionMarkerPublisher&&)      = delete;
    ~RobotCollisionMarkerPublisher()                                               = default;

    void configure(bool self_collision, bool plane_collision, bool ground_collision, bool roof_collision, bool static_obstacle_collision,
                   bool dynamic_obstacle_collision, bool human_collision, bool publish_distances);

    void publish();

 private:
    using ObstacleList = robot_obstacle::ObstacleList;
    using Common       = robot_misc::Common;
    using RobotUtility = robot_misc::RobotUtility;
    using Obstacle     = robot_misc::Obstacle;
    using Plane        = robot_misc::Plane;
    using Human        = robot_misc::Human;

    ObstacleList _obstacle_manager;
    RobotCollision::UPtr _robot_collision;
    RobotUtility::Ptr _robot_utility;

    std::vector<double> _joint_states;
    std::string _joint_state_topic;
    std::string _obstacle_topic;

    ros::Publisher _line_marker_pub;
    ros::Publisher _point_marker_pub;
    ros::Publisher _distance_pub;

    ros::Subscriber _joint_state_subscriber;
    ros::Subscriber _obstacle_subscriber;

    bool _first_joint_state          = true;
    bool _self_collision             = true;
    bool _plane_collision            = true;
    bool _ground_collision           = true;
    bool _roof_collision             = true;
    bool _static_obstacle_collision  = true;
    bool _dynamic_obstacle_collision = true;
    bool _human_collision            = true;
    bool _publish_distances          = true;
    int _visualization_id_spacing    = 4;

    double _self_collision_distance;
    double _static_obstacle_distance;
    double _dynamic_obstacle_distance;
    double _ground_distance;
    double _roof_distance;
    double _plane_distance;
    std::vector<double> _human_distances;
    std::vector<double> _human_distances_segments;

    void obstacleCallback(const mhp_robot::MsgObstacleListConstPtr& msg);
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);

    void publishMarker(const Eigen::Ref<const Eigen::Vector3d>& start, const Eigen::Ref<const Eigen::Vector3d>& end, double distance,
                       const robot_misc::Color& color, const std::string& label, int id);
    void publishDistances();

    void updateSelfCollisionMarker();
    void updateGroundCollisionMarker();
    void updateRoofCollisionMarker();
    void updatePlaneCollisionMarker();
    void updateStaticObstacleCollisionMarker();
    void updateDynamicObstacleCollisionMarker();
    void updateHumanCollisionMarker();
};

}  // namespace robot_collision
}  // namespace mhp_robot

#endif  // ROBOT_COLLISION_MARKER_PUBLISHER_H
