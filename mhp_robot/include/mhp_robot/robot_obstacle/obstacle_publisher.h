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

#ifndef OBSTACLE_PUBLISHER_H
#define OBSTACLE_PUBLISHER_H

#include <mhp_robot/robot_misc/color.h>
#include <mhp_robot/robot_misc/human.h>
#include <mhp_robot/robot_misc/obstacle.h>
#include <mhp_robot/robot_misc/plane.h>
#include <mhp_robot/robot_misc/utility_object.h>
#include <ros/publisher.h>
#include <ros/service.h>
#include <visualization_msgs/Marker.h>

namespace mhp_robot {
namespace robot_obstacle {

class ObstaclePublisher
{
 public:
    using Ptr  = std::shared_ptr<ObstaclePublisher>;
    using UPtr = std::unique_ptr<ObstaclePublisher>;

    ObstaclePublisher(const std::string& prefix = "");

    ObstaclePublisher(const ObstaclePublisher&)            = delete;
    ObstaclePublisher(ObstaclePublisher&&)                 = default;
    ObstaclePublisher& operator=(const ObstaclePublisher&) = delete;
    ObstaclePublisher& operator=(ObstaclePublisher&&)      = default;
    ~ObstaclePublisher()                                   = default;

    void publishObstacles(const std::map<int, robot_misc::Obstacle>& static_obstacles, const std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                          const std::map<int, robot_misc::Human>& humans, const std::map<int, robot_misc::UtilityObject>& utility_objects,
                          const std::map<int, robot_misc::Plane>& planes);

    void publishMarker(const std::map<int, robot_misc::Obstacle>& static_obstacles, const std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                       const std::map<int, robot_misc::Human>& humans, const std::map<int, robot_misc::UtilityObject>& utility_objects,
                       const std::map<int, robot_misc::Plane>& planes);

    void publishGazebo(const std::map<int, robot_misc::Obstacle>& static_obstacles, const std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                       const std::map<int, robot_misc::Human>& humans, const std::map<int, robot_misc::UtilityObject>& utility_objects,
                       const std::map<int, robot_misc::Plane>& planes);

    bool _publish_predicted_human        = true;
    bool _publish_uncertainty_body_parts = true;

 private:
    using BoundingBoxType = robot_misc::BoundingBoxType;
    using Color           = robot_misc::Color;
    using Obstacle        = robot_misc::Obstacle;
    using UtilityObject   = robot_misc::UtilityObject;
    using Plane           = robot_misc::Plane;
    using Human           = robot_misc::Human;

    ros::NodeHandle _n;

    bool _gazebo                  = false;
    bool _gazebo_initialized      = false;
    int _visualization_id_spacing = 10;
    double _color_alpha           = 1.0;

    // std::map<int, robot_misc::Human> _predicted_humans;

    ros::Publisher _static_marker_pub;
    ros::Publisher _dynamic_marker_pub;
    ros::Publisher _human_marker_pub;
    ros::Publisher _predicted_human_marker_pub;
    ros::Publisher _utility_marker_pub;
    ros::Publisher _plane_marker_pub;

    ros::Publisher _obstacle_publisher;
    ros::Publisher _gazebo_model_state_publisher;
    std::map<int, ros::Publisher> _obstacle_prediction_publisher;

    std::string exec(const char* cmd) const;

    void initializeGazebo(const std::map<int, Obstacle>& static_obstacles, const std::map<int, Obstacle>& dynamic_obstacles,
                          const std::map<int, Human>& humans, const std::map<int, Plane>& planes);

    visualization_msgs::Marker createSphereMarker(const Obstacle& obstacle, const Color& color, const double& timeStep = 0) const;
    //    visualization_msgs::Marker createSphereMarker(const Obstacle& obstacle, const Color& color, const double& timeStep, const double& lifetime)
    //    const;
    std::vector<visualization_msgs::Marker> createUncertaintySphereMarker(const Obstacle& obstacle, const Color& color,
                                                                          const double& timeStep = 0) const;
    std::vector<visualization_msgs::Marker> createCylinderMarker(const Obstacle& obstacle, const Color& color, const double& timeStep = 0) const;
    //    std::vector<visualization_msgs::Marker> createCylinderMarker(const Obstacle& obstacle, const Color& color, const double& timeStep,
    //                                                                 const double& lifetime) const;
    std::vector<visualization_msgs::Marker> createUncertaintyCylinderMarker(const Obstacle& obstacle, const Color& color,
                                                                            const double& timeStep = 0) const;
    visualization_msgs::Marker createPlaneMarker(const Plane& plane, const Color& color, const double& timeStep = 0) const;
    std::vector<visualization_msgs::Marker> createCubeMarker(const Obstacle& obstacle, const Color& color, const double& timeStep = 0) const;
    std::vector<visualization_msgs::Marker> createExtrudedCubeMarker(const Obstacle& obstacle, const Color& color, const double& timeStep = 0) const;

    void staticObstacleMarkerUpdate(const std::map<int, Obstacle>& static_obstacles);
    void dynamicObstacleMarkerUpdate(const std::map<int, Obstacle>& dynamic_obstacles);
    void humanMarkerUpdate(const std::map<int, Human>& humans);
    void predictedHumanMarkerUpdate(const std::map<int, Human>& predictedHumans, double timeStep);
    void utilityObjectMarkerUpdate(const std::map<int, UtilityObject>& utility_objects);
    void planeMarkerUpdate(const std::map<int, Plane>& planes);

    void gazeboModelUpdate(const std::map<int, Obstacle>& dynamic_obstacles, const std::map<int, Human>& humans);
    void spawnStaticObstacles(ros::ServiceClient& gazebo_service_client, const std::map<int, Obstacle>& static_obstacles);
    void spawnDynamicObstacles(ros::ServiceClient& gazebo_service_client, const std::map<int, Obstacle>& dynamic_obstacles);
    void spawnHumans(ros::ServiceClient& gazebo_service_client, const std::map<int, Human>& humans);
    void spawnPlanes(ros::ServiceClient& gazebo_service_client, const std::map<int, Plane>& planes);
    void spawnGazeboModel(const std::stringstream& name, const std::string& description, ros::ServiceClient& gazebo_service_client,
                          const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose);
};

}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // OBSTACLE_PUBLISHER_H
