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

#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SpawnModelRequest.h>
#include <gazebo_msgs/SpawnModelResponse.h>
#include <mhp_robot/MsgHuman.h>
#include <mhp_robot/MsgObstacleList.h>
#include <mhp_robot/MsgPlane.h>
#include <mhp_robot/MsgUtilityObject.h>
#include <mhp_robot/robot_misc/common.h>
#include <mhp_robot/robot_obstacle/obstacle_publisher.h>
#include <nav_msgs/Path.h>
#include <ros/io.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace mhp_robot {
namespace robot_obstacle {

ObstaclePublisher::ObstaclePublisher(const std::string& prefix) : _n("~")
{
    // Setup static marker publisher
    _static_marker_pub = _n.advertise<visualization_msgs::Marker>(prefix + "static_obstacle_marker", 1000);

    // Setup dynamic marker publisher
    _dynamic_marker_pub = _n.advertise<visualization_msgs::Marker>(prefix + "dynamic_obstacle_marker", 1000);

    // Setup human marker publisher
    _human_marker_pub = _n.advertise<visualization_msgs::Marker>(prefix + "human_marker", 1000);

    // Setup utility object marker publisher
    _utility_marker_pub = _n.advertise<visualization_msgs::Marker>(prefix + "utility_object_marker", 1000);

    // Setup planes marker publisher
    _plane_marker_pub = _n.advertise<visualization_msgs::Marker>(prefix + "plane_marker", 1000);

    // Setup obstacle publisher
    _obstacle_publisher = _n.advertise<MsgObstacleList>(prefix + "obstacles", 1000);

    // Setup predicted Human and Publisher
    _predicted_human_marker_pub = _n.advertise<visualization_msgs::Marker>(prefix + "predicted_human_marker", 1000);
}

void ObstaclePublisher::publishObstacles(const std::map<int, Obstacle>& static_obstacles, const std::map<int, Obstacle>& dynamic_obstacles,
                                         const std::map<int, Human>& humans, const std::map<int, UtilityObject>& utility_objects,
                                         const std::map<int, Plane>& planes)
{
    MsgObstacleList list_msg;

    // Publish static obstacles
    for (auto const& obs : static_obstacles)
    {
        MsgObstacle obstacle_msg;
        robot_misc::Common::generateObstacleMsg(obs.second, obstacle_msg);
        list_msg.static_obstacles.push_back(obstacle_msg);
    }

    // Publish dynamic obstacles
    for (auto const& obs : dynamic_obstacles)
    {
        MsgObstacle obstacle_msg;
        robot_misc::Common::generateObstacleMsg(obs.second, obstacle_msg);
        list_msg.dynamic_obstacles.push_back(obstacle_msg);
    }

    // Publish humans
    for (auto const& hum : humans)
    {
        MsgHuman human_msg;
        robot_misc::Common::generateHumanMsg(hum.second, human_msg);
        list_msg.humans.push_back(human_msg);
    }

    for (auto const& obj : utility_objects)
    {
        MsgUtilityObject object_msg;
        robot_misc::Common::generateUtilityObjectMsg(obj.second, object_msg);
        list_msg.utility_objects.push_back(object_msg);
    }

    for (auto const& p : planes)
    {
        MsgPlane plane_msg;
        robot_misc::Common::generatePlaneMsg(p.second, plane_msg);
        list_msg.planes.push_back(plane_msg);
    }

    list_msg.header.stamp    = ros::Time::now();
    list_msg.header.frame_id = "0";
    _obstacle_publisher.publish(list_msg);
}

void ObstaclePublisher::publishMarker(const std::map<int, Obstacle>& static_obstacles, const std::map<int, Obstacle>& dynamic_obstacles,
                                      const std::map<int, Human>& humans, const std::map<int, UtilityObject>& utility_objects,
                                      const std::map<int, Plane>& planes)
{
    staticObstacleMarkerUpdate(static_obstacles);
    dynamicObstacleMarkerUpdate(dynamic_obstacles);
    humanMarkerUpdate(humans);
    if (_publish_predicted_human)
    {
        // 3 seconds in the future --> keep in mind that if you use less extrapolation steps a ZOH fills the extrapolations up
        predictedHumanMarkerUpdate(humans, 3);
    }
    utilityObjectMarkerUpdate(utility_objects);
    planeMarkerUpdate(planes);
}

void ObstaclePublisher::publishGazebo(const std::map<int, Obstacle>& static_obstacles, const std::map<int, Obstacle>& dynamic_obstacles,
                                      const std::map<int, Human>& humans, const std::map<int, UtilityObject>& utility_objects,
                                      const std::map<int, Plane>& planes)
{
    // Initialize if needed
    if (!_gazebo_initialized)
    {
        initializeGazebo(static_obstacles, dynamic_obstacles, humans, planes);
    }

    // Update gazebo models
    gazeboModelUpdate(dynamic_obstacles, humans);
}

void ObstaclePublisher::initializeGazebo(const std::map<int, Obstacle>& static_obstacles, const std::map<int, Obstacle>& dynamic_obstacles,
                                         const std::map<int, Human>& humans, const std::map<int, Plane>& planes)
{
    ROS_INFO("[ObstaclePublisher] Initializing...");

    // Check if gazebo service is available
    ROS_INFO("[ObstaclePublisher] Waiting for Gazebo...");
    _gazebo = ros::service::waitForService("/gazebo/spawn_urdf_model", ros::Duration(5));

    if (_gazebo)
    {
        ROS_INFO("[ObstaclePublisher] Using Gazebo");

        // Setup gazebo services
        ros::ServiceClient gazebo_service_client = _n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
        _gazebo_model_state_publisher            = _n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);

        // Spawn gazebo obstacles
        spawnStaticObstacles(gazebo_service_client, static_obstacles);
        spawnDynamicObstacles(gazebo_service_client, dynamic_obstacles);
        spawnHumans(gazebo_service_client, humans);
        spawnPlanes(gazebo_service_client, planes);
    }
    else
    {
        ROS_INFO("[ObstaclePublisher] Gazebo not found. Not using Gazebo");
    }

    ROS_INFO("[ObstaclePublisher] Initializing done");
    _gazebo_initialized = true;
}

void ObstaclePublisher::staticObstacleMarkerUpdate(const std::map<int, Obstacle>& static_obstacles)
{
    int i = 0;
    for (auto const& obs : static_obstacles)
    {
        const Obstacle& obstacle = obs.second;

        if (obstacle.bounding_box.type == BoundingBoxType::SPHERE)
        {
            visualization_msgs::Marker sphere = createSphereMarker(obstacle, Color(0, 1, 0, _color_alpha));

            sphere.id = i * _visualization_id_spacing;
            _static_marker_pub.publish(sphere);
        }
        else if (obstacle.bounding_box.type == BoundingBoxType::CYLINDER)
        {
            std::vector<visualization_msgs::Marker> marker = createCylinderMarker(obstacle, Color(0, 1, 0, _color_alpha));

            for (int j = 0; j < marker.size(); ++j)
            {
                marker[j].id = i * _visualization_id_spacing + j;
                _static_marker_pub.publish(marker[j]);
            }
        }
        else if (obstacle.bounding_box.type == BoundingBoxType::BOX)
        {
            std::vector<visualization_msgs::Marker> marker = createCubeMarker(obstacle, Color(0, 1, 0, _color_alpha));
            for (int j = 0; j < marker.size(); ++j)
            {
                marker[j].id = i * _visualization_id_spacing + j;
                _static_marker_pub.publish(marker[j]);
            }
        }
        else if (obstacle.bounding_box.type == BoundingBoxType::EBOX)
        {
            std::vector<visualization_msgs::Marker> marker = createExtrudedCubeMarker(obstacle, Color(0, 1, 0, _color_alpha));
            for (int j = 0; j < marker.size(); ++j)
            {
                marker[j].id = i * _visualization_id_spacing + j;
                _static_marker_pub.publish(marker[j]);
            }
        }
        else
        {

            ROS_ERROR("ObstaclePublisher: Static obstacle bbox type not supported");
        }
        ++i;
    }
}

void ObstaclePublisher::dynamicObstacleMarkerUpdate(const std::map<int, Obstacle>& dynamic_obstacles)
{
    int i = 0;
    std::map<int, ros::Publisher>::iterator it;

    for (auto const& obs : dynamic_obstacles)
    {
        const Obstacle& obstacle = obs.second;

        Color color(0, 1, 0, _color_alpha);
        if (obstacle.temporary_static)
        {
            // Overwrite color
            color.r = 0.0f;
            color.g = 1.0f;
            color.b = 1.0f;
        }

        if (obstacle.bounding_box.type == BoundingBoxType::SPHERE)
        {
            visualization_msgs::Marker sphere = createSphereMarker(obstacle, color);

            sphere.id = i * _visualization_id_spacing;
            _dynamic_marker_pub.publish(sphere);
        }
        else if (obstacle.bounding_box.type == BoundingBoxType::CYLINDER)
        {
            std::vector<visualization_msgs::Marker> marker = createCylinderMarker(obstacle, color);

            for (int j = 0; j < marker.size(); ++j)
            {
                marker[j].id = i * _visualization_id_spacing + j;
                _dynamic_marker_pub.publish(marker[j]);
            }
        }
        else if (obstacle.bounding_box.type == BoundingBoxType::BOX)
        {
            std::vector<visualization_msgs::Marker> marker = createCubeMarker(obstacle, Color(0, 1, 0, _color_alpha));
            for (int j = 0; j < marker.size(); ++j)
            {
                marker[j].id = i * _visualization_id_spacing + j;
                _dynamic_marker_pub.publish(marker[j]);
            }
        }
        else if (obstacle.bounding_box.type == BoundingBoxType::EBOX)
        {
            std::vector<visualization_msgs::Marker> marker = createExtrudedCubeMarker(obstacle, Color(0, 1, 0, _color_alpha));
            for (int j = 0; j < marker.size(); ++j)
            {
                marker[j].id = i * _visualization_id_spacing + j;
                _dynamic_marker_pub.publish(marker[j]);
            }
        }
        else
        {
            ROS_ERROR("ObstaclePublisher: Dynamic obstacle bbox type not supported");
        }

        nav_msgs::Path path_msg;
        for (auto const& pose : obs.second.state._poses)
        {
            geometry_msgs::PoseStamped pose_stamped_msg;

            robot_misc::Common::poseEigenToMsg(pose, pose_stamped_msg.pose);
            path_msg.poses.push_back(pose_stamped_msg);
        }

        path_msg.header.stamp    = ros::Time::now();
        path_msg.header.frame_id = "world";

        // Setup prediction publisher
        it = _obstacle_prediction_publisher.find(obs.second.id);
        if (it == _obstacle_prediction_publisher.end())
        {
            // Create publisher for that obstacle
            ros::Publisher pub = _n.advertise<nav_msgs::Path>("obstacle_" + std::to_string(obs.second.id) + "_path", 1);
            it                 = (_obstacle_prediction_publisher.insert(std::pair<int, ros::Publisher>(obs.second.id, pub))).first;
        }

        it->second.publish(path_msg);
        ++i;
    }
}

void ObstaclePublisher::humanMarkerUpdate(const std::map<int, Human>& humans)
{
    int i = 0;
    for (auto const& hum : humans)
    {
        const Human& human = hum.second;

        for (auto const& body : human._body_parts)
        {
            if (body.second.bounding_box.type == BoundingBoxType::SPHERE)
            {
                visualization_msgs::Marker sphere = createSphereMarker(body.second, Color(0, 1, 0, _color_alpha));

                sphere.id = i * _visualization_id_spacing;
                _human_marker_pub.publish(sphere);
            }
            else if (body.second.bounding_box.type == BoundingBoxType::CYLINDER)
            {
                std::vector<visualization_msgs::Marker> marker = createCylinderMarker(body.second, Color(0, 1, 0, _color_alpha));

                for (int j = 0; j < marker.size(); ++j)
                {
                    marker[j].id = i * _visualization_id_spacing + j;
                    _human_marker_pub.publish(marker[j]);
                }
            }
            else if (body.second.bounding_box.type == BoundingBoxType::BOX)
            {
                std::vector<visualization_msgs::Marker> marker = createCubeMarker(body.second, Color(0, 1, 0, _color_alpha));
                for (int j = 0; j < marker.size(); ++j)
                {
                    marker[j].id = i * _visualization_id_spacing + j;
                    _human_marker_pub.publish(marker[j]);
                }
            }
            else if (body.second.bounding_box.type == BoundingBoxType::EBOX)
            {
                std::vector<visualization_msgs::Marker> marker = createExtrudedCubeMarker(body.second, Color(0, 1, 0, _color_alpha));
                for (int j = 0; j < marker.size(); ++j)
                {
                    marker[j].id = i * _visualization_id_spacing + j;
                    _human_marker_pub.publish(marker[j]);
                }
            }
            else if (body.second.bounding_box.type == BoundingBoxType::NOTYPE)
            {
                //                ROS_INFO("ObstaclePublisher: No Type to display for %s", body.second.name.c_str());
            }
            else
            {

                ROS_ERROR("ObstaclePublisher: Human bbox type not supported %s", body.second.name.c_str());
            }
            ++i;
        }
    }
}

void ObstaclePublisher::predictedHumanMarkerUpdate(const std::map<int, Human>& humans, double timeStep)
{
    int i = 0;

    // Clean up prediction marker topic
    visualization_msgs::Marker deleteMarker;
    deleteMarker.header.frame_id = "world";
    deleteMarker.action          = visualization_msgs::Marker::DELETEALL;
    _predicted_human_marker_pub.publish(deleteMarker);

    // Iterate over humans
    for (auto const& hum : humans)
    {
        const Human& human = hum.second;
        // Iterate over body parts and publish predicted future position at timeStep
        for (auto const& body : human._body_parts)
        {
            if (body.second.bounding_box.type == BoundingBoxType::SPHERE)
            {
                visualization_msgs::Marker sphere = createSphereMarker(body.second, Color(1, 0, 0, _color_alpha), timeStep);

                sphere.id = i * _visualization_id_spacing;
                _predicted_human_marker_pub.publish(sphere);
            }
            else if (body.second.bounding_box.type == BoundingBoxType::CYLINDER)
            {
                std::vector<visualization_msgs::Marker> marker = createCylinderMarker(body.second, Color(1, 0, 0, _color_alpha), timeStep);

                for (int j = 0; j < marker.size(); ++j)
                {
                    marker[j].id = i * _visualization_id_spacing + j;
                    _predicted_human_marker_pub.publish(marker[j]);
                }
            }
            else if (body.second.bounding_box.type == BoundingBoxType::BOX)
            {
                std::vector<visualization_msgs::Marker> marker = createCubeMarker(body.second, Color(1, 0, 0, _color_alpha), timeStep);
                for (int j = 0; j < marker.size(); ++j)
                {
                    marker[j].id = i * _visualization_id_spacing + j;
                    _predicted_human_marker_pub.publish(marker[j]);
                }
            }
            else if (body.second.bounding_box.type == BoundingBoxType::EBOX)
            {
                std::vector<visualization_msgs::Marker> marker = createExtrudedCubeMarker(body.second, Color(1, 0, 0, _color_alpha), timeStep);
                for (int j = 0; j < marker.size(); ++j)
                {
                    marker[j].id = i * _visualization_id_spacing + j;
                    _predicted_human_marker_pub.publish(marker[j]);
                }
            }
            else if (body.second.bounding_box.type == BoundingBoxType::NOTYPE)
            {
                //                ROS_INFO("ObstaclePublisher: No Type to display for %s", body.second.name.c_str());
            }
            else
            {

                ROS_ERROR("ObstaclePublisher: Human bbox type not supported %s", body.second.name.c_str());
            }

            ++i;
        }
        // If uncertainty body parts from skeleton splitting should be plotted
        if (_publish_uncertainty_body_parts)
        {
            // Iterate over body parts and plot uncertainty elements
            for (auto const& body : human._body_parts)
            {
                if (body.second.bounding_box.type == BoundingBoxType::SPHERE)
                {
                    std::vector<visualization_msgs::Marker> spheres =
                        createUncertaintySphereMarker(body.second, Color(1, 0, 0, _color_alpha), timeStep);
                    for (int j = 0; j < spheres.size(); ++j)
                    {
                        spheres[j].id = i * _visualization_id_spacing + j;
                        _predicted_human_marker_pub.publish(spheres[j]);
                    }
                }
                else if (body.second.bounding_box.type == BoundingBoxType::CYLINDER)
                {
                    std::vector<visualization_msgs::Marker> marker =
                        createUncertaintyCylinderMarker(body.second, Color(1, 0, 0, _color_alpha), timeStep);

                    for (int j = 0; j < marker.size(); ++j)
                    {
                        marker[j].id = i * _visualization_id_spacing + j;
                        _predicted_human_marker_pub.publish(marker[j]);
                    }
                }
                else
                {
                    ROS_ERROR("ObstaclePublisher: Human bbox type not supported %s", body.second.name.c_str());
                }
                ++i;
            }
        }
    }
}

void ObstaclePublisher::utilityObjectMarkerUpdate(const std::map<int, UtilityObject>& utility_objects)
{
    int i = 0;
    for (auto const& obj : utility_objects)
    {
        const UtilityObject& util_object = obj.second;

        Color color(0, 0, 1, _color_alpha);
        if (util_object.held)
        {
            color.r = 1.0f;
            color.g = 0.0f;
            color.b = 0.0f;
        }

        if (util_object.bounding_box.type == BoundingBoxType::SPHERE)
        {
            visualization_msgs::Marker sphere = createSphereMarker(util_object, color);

            sphere.id = i * _visualization_id_spacing;
            _utility_marker_pub.publish(sphere);
        }
        else if (util_object.bounding_box.type == BoundingBoxType::CYLINDER)
        {
            std::vector<visualization_msgs::Marker> marker = createCylinderMarker(util_object, color);

            for (int j = 0; j < marker.size(); ++j)
            {
                marker[j].id = i * _visualization_id_spacing + j;
                _utility_marker_pub.publish(marker[j]);
            }
        }
        else if (util_object.bounding_box.type == BoundingBoxType::BOX)
        {
            std::vector<visualization_msgs::Marker> marker = createCubeMarker(util_object, Color(0, 1, 0, _color_alpha));
            for (int j = 0; j < marker.size(); ++j)
            {
                marker[j].id = i * _visualization_id_spacing + j;
                _utility_marker_pub.publish(marker[j]);
            }
        }
        else if (util_object.bounding_box.type == BoundingBoxType::EBOX)
        {
            std::vector<visualization_msgs::Marker> marker = createExtrudedCubeMarker(util_object, Color(0, 1, 0, _color_alpha));
            for (int j = 0; j < marker.size(); ++j)
            {
                marker[j].id = i * _visualization_id_spacing + j;
                _utility_marker_pub.publish(marker[j]);
            }
        }
        else
        {
            ROS_ERROR("ObstaclePublisher: Utility bbox type not supported");
        }
        ++i;
    }
}

void ObstaclePublisher::planeMarkerUpdate(const std::map<int, Plane>& planes)
{
    int i = 0;
    for (auto const& plane : planes)
    {
        const Plane& p = plane.second;

        visualization_msgs::Marker plane_marker = createPlaneMarker(p, Color(0.5, 0.5, 0.5, _color_alpha));

        plane_marker.id = i;

        _plane_marker_pub.publish(plane_marker);
        ++i;
    }
}

void ObstaclePublisher::gazeboModelUpdate(const std::map<int, Obstacle>& dynamic_obstacles, const std::map<int, Human>& humans)
{
    if (!_gazebo) return;

    gazebo_msgs::ModelState msg;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (auto const& obs : dynamic_obstacles)
    {
        // Skip update if not requested
        if (!obs.second.gazebo) continue;

        std::stringstream name;
        name << "dynamic_obstacle_" << obs.second.id;

        msg.model_name      = name.str();
        msg.reference_frame = "world";

        if (obs.second.bounding_box.type == robot_misc::BoundingBoxType::SPHERE)
        {
            T.noalias() = obs.second.state.getPose() * obs.second.bounding_box.T;
        }
        else if (obs.second.bounding_box.type == robot_misc::BoundingBoxType::CYLINDER)
        {
            // Wrap urdf cylinder convention
            T.noalias() = obs.second.state.getPose() * obs.second.bounding_box.T * robot_misc::Common::roty(M_PI / 2.0);
        }
        else if (obs.second.bounding_box.type == robot_misc::BoundingBoxType::BOX)
        {
            T.noalias() = obs.second.state.getPose() * obs.second.bounding_box.T;
        }
        else if (obs.second.bounding_box.type == robot_misc::BoundingBoxType::EBOX)
        {
            T.noalias() = obs.second.state.getPose() * obs.second.bounding_box.T;
        }

        robot_misc::Common::poseEigenToMsg(T, msg.pose);

        _gazebo_model_state_publisher.publish(msg);
    }

    for (auto const& human : humans)
    {
        // Skip update if not requested
        if (!human.second._gazebo) continue;

        for (auto const& body : human.second._body_parts)
        {
            std::stringstream name;
            name << "human_" << human.second._id << "_" << body.second.name;

            msg.model_name      = name.str();
            msg.reference_frame = "world";

            if (body.second.bounding_box.type == robot_misc::BoundingBoxType::SPHERE)
            {
                T.noalias() = body.second.state.getPose() * body.second.bounding_box.T;
            }
            else if (body.second.bounding_box.type == robot_misc::BoundingBoxType::CYLINDER)
            {
                // Wrap urdf cylinder convention
                T.noalias() = body.second.state.getPose() * body.second.bounding_box.T * robot_misc::Common::roty(M_PI / 2.0);
            }
            else if (body.second.bounding_box.type == robot_misc::BoundingBoxType::BOX)
            {
                T.noalias() = body.second.state.getPose() * body.second.bounding_box.T;
            }
            else if (body.second.bounding_box.type == robot_misc::BoundingBoxType::EBOX)
            {
                T.noalias() = body.second.state.getPose() * body.second.bounding_box.T;
            }

            robot_misc::Common::poseEigenToMsg(T, msg.pose);

            _gazebo_model_state_publisher.publish(msg);
        }
    }
}

std::vector<visualization_msgs::Marker> ObstaclePublisher::createCylinderMarker(const Obstacle& obstacle, const Color& color,
                                                                                const double& timeStep) const
{
    visualization_msgs::Marker cylinder;

    cylinder.header.frame_id = "world";
    cylinder.action          = visualization_msgs::Marker::ADD;
    cylinder.lifetime        = ros::Duration(0.0);

    cylinder.scale.x = 2 * obstacle.bounding_box.getRadius(timeStep);
    cylinder.scale.y = 2 * obstacle.bounding_box.getRadius(timeStep);
    cylinder.scale.z = obstacle.bounding_box.length_x;

    cylinder.color.r = color.r;
    cylinder.color.g = color.g;
    cylinder.color.b = color.b;
    cylinder.color.a = color.a;

    std::ostringstream s;
    if (obstacle.group_id == 0)
    {
        s << obstacle.name << " (" << obstacle.id << ")";
    }
    else
    {
        s << "Group " << obstacle.group_id;
    }
    cylinder.ns = s.str();

    cylinder.header.stamp = ros::Time::now();
    cylinder.type         = visualization_msgs::Marker::CYLINDER;

    // Transform into start point and let z axis point into cylinder direction
    Eigen::Matrix4d T;
    T.noalias() = obstacle.state.getPose(timeStep) * obstacle.bounding_box.T * robot_misc::Common::roty(M_PI / 2.0);

    //    if (obstacle.name.compare("Neck") == 0)
    //    {
    //        std::cout << "Body part: " << obstacle.name << "\n with Pose: \n"
    //                  << obstacle.state.getPose(timeStep) << "\n at time step: " << timeStep << "\n"
    //                  << "and at timestep 0: \n"
    //                  << obstacle.state.getPose() << "\n"
    //                  << std::endl;
    //    }

    // Calculate origin of the cylinder marker
    Eigen::Vector3d c;
    c << 0, 0, obstacle.bounding_box.length_x / 2.0;
    c = T.block<3, 3>(0, 0) * c;

    cylinder.pose.position.x = T(0, 3) + c(0);
    cylinder.pose.position.y = T(1, 3) + c(1);
    cylinder.pose.position.z = T(2, 3) + c(2);

    geometry_msgs::Pose p;
    robot_misc::Common::poseEigenToMsg(T, p);
    cylinder.pose.orientation = p.orientation;

    // Spheres
    visualization_msgs::Marker start = createSphereMarker(obstacle, color, timeStep);
    visualization_msgs::Marker end   = createSphereMarker(obstacle, color, timeStep);

    // Adjust position of end point sphere
    Eigen::Vector3d d;
    d << 0, 0, obstacle.bounding_box.length_x;
    d = T.block<3, 3>(0, 0) * d;

    end.pose.position.x = T(0, 3) + d(0);
    end.pose.position.y = T(1, 3) + d(1);
    end.pose.position.z = T(2, 3) + d(2);

    return {start, cylinder, end};
}

std::vector<visualization_msgs::Marker> ObstaclePublisher::createUncertaintyCylinderMarker(const ObstaclePublisher::Obstacle& obstacle,
                                                                                           const ObstaclePublisher::Color& color,
                                                                                           const double& timeStep) const
{
    std::vector<visualization_msgs::Marker> allCylinders(obstacle.uncertainty_states.size());
    std::vector<Eigen::Matrix4d> tmpT(obstacle.uncertainty_states.size());
    for (int cnt = 0; cnt < allCylinders.size(); ++cnt)
    {
        visualization_msgs::Marker cylinder;

        cylinder.header.frame_id = "world";
        cylinder.action          = visualization_msgs::Marker::ADD;
        cylinder.lifetime        = ros::Duration(0.0);

        cylinder.scale.x = 2 * obstacle.bounding_box.getRadius(timeStep);
        cylinder.scale.y = 2 * obstacle.bounding_box.getRadius(timeStep);
        cylinder.scale.z = obstacle.bounding_box.length_x;

        cylinder.color.r = color.r;
        cylinder.color.g = color.g;
        cylinder.color.b = color.b;
        cylinder.color.a = color.a;

        std::ostringstream s;
        if (obstacle.group_id == 0)
        {
            s << obstacle.name << " (" << obstacle.id << ")";
        }
        else
        {
            s << "Group " << obstacle.group_id;
        }
        cylinder.ns = s.str();

        cylinder.header.stamp = ros::Time::now();
        cylinder.type         = visualization_msgs::Marker::CYLINDER;

        // Transform into start point and let z axis point into cylinder direction
        Eigen::Matrix4d T;
        T.noalias() = obstacle.uncertainty_states.at(cnt).getPose(timeStep) * obstacle.bounding_box.T * robot_misc::Common::roty(M_PI / 2.0);

        // Calculate origin of the cylinder marker
        Eigen::Vector3d c;
        c << 0, 0, obstacle.bounding_box.length_x / 2.0;
        c = T.block<3, 3>(0, 0) * c;

        cylinder.pose.position.x = T(0, 3) + c(0);
        cylinder.pose.position.y = T(1, 3) + c(1);
        cylinder.pose.position.z = T(2, 3) + c(2);

        geometry_msgs::Pose p;
        robot_misc::Common::poseEigenToMsg(T, p);
        cylinder.pose.orientation = p.orientation;

        allCylinders[cnt] = cylinder;
        tmpT[cnt]         = T;
    }
    // Spheres
    std::vector<visualization_msgs::Marker> starts = createUncertaintySphereMarker(obstacle, color, timeStep);
    std::vector<visualization_msgs::Marker> ends   = createUncertaintySphereMarker(obstacle, color, timeStep);

    // Adjust position of end point sphere
    if (ends.size() == allCylinders.size())
    {
        for (int cnt = 0; cnt < allCylinders.size(); ++cnt)
        {
            Eigen::Vector3d d;
            d << 0, 0, obstacle.bounding_box.length_x;
            d = tmpT[cnt].block<3, 3>(0, 0) * d;

            ends[cnt].pose.position.x = tmpT[cnt](0, 3) + d(0);
            ends[cnt].pose.position.y = tmpT[cnt](1, 3) + d(1);
            ends[cnt].pose.position.z = tmpT[cnt](2, 3) + d(2);
        }
    }
    std::vector<visualization_msgs::Marker> results;
    results.insert(results.end(), starts.begin(), starts.end());
    results.insert(results.end(), allCylinders.begin(), allCylinders.end());
    results.insert(results.end(), ends.begin(), ends.end());

    return results;
}  // namespace robot_obstacle

// std::vector<visualization_msgs::Marker> ObstaclePublisher::createCylinderMarker(const ObstaclePublisher::Obstacle& obstacle,
//                                                                                const ObstaclePublisher::Color& color, const double& timeStep,
//                                                                                const double& lifetime) const
//{
//    std::vector<visualization_msgs::Marker> markers = createCylinderMarker(obstacle, color, timeStep);
//    for (auto var : markers)
//    {
//        var.header.stamp = ros::Time::now();
//        var.lifetime     = ros::Duration(lifetime);
//    }
//    return markers;
//}

visualization_msgs::Marker ObstaclePublisher::createPlaneMarker(const Plane& plane, const Color& color, const double& timeStep) const
{
    visualization_msgs::Marker plane_marker;

    plane_marker.header.frame_id = "world";
    plane_marker.action          = visualization_msgs::Marker::ADD;
    plane_marker.lifetime        = ros::Duration(0.0);

    plane_marker.scale.x = 4;
    plane_marker.scale.y = 2;
    plane_marker.scale.z = 0.02;

    plane_marker.color.r = color.r;
    plane_marker.color.g = color.g;
    plane_marker.color.b = color.b;
    plane_marker.color.a = color.a;

    std::ostringstream s;
    if (plane.group_id == 0)
    {
        s << plane.name << " (" << plane.id << ")";
    }
    else
    {
        s << "Group " << plane.group_id;
    }
    plane_marker.ns = s.str();

    plane_marker.header.stamp = ros::Time::now();
    plane_marker.type         = visualization_msgs::Marker::CUBE;

    plane_marker.pose.position.x = plane.q(0);
    plane_marker.pose.position.y = plane.q(1);
    plane_marker.pose.position.z = plane.q(2);

    Eigen::Quaterniond quat(robot_misc::Common::poseFromPlane(plane.q, plane.n).block<3, 3>(0, 0));
    quat.normalize();
    plane_marker.pose.orientation.w = quat.w();
    plane_marker.pose.orientation.x = quat.x();
    plane_marker.pose.orientation.y = quat.y();
    plane_marker.pose.orientation.z = quat.z();

    return plane_marker;
}

std::vector<visualization_msgs::Marker> ObstaclePublisher::createCubeMarker(const Obstacle& obstacle, const Color& color,
                                                                            const double& timeStep) const
{
    visualization_msgs::Marker box;

    box.header.frame_id = "world";
    box.action          = visualization_msgs::Marker::ADD;
    box.lifetime        = ros::Duration(0.0);

    box.scale.x = obstacle.bounding_box.length_x;
    box.scale.y = obstacle.bounding_box.length_y;
    box.scale.z = obstacle.bounding_box.radius * 2;

    box.color.r = color.r;
    box.color.g = color.g;
    box.color.b = color.b;
    box.color.a = color.a;

    std::ostringstream s;
    if (obstacle.group_id == 0)
    {
        s << obstacle.name << " (" << obstacle.id << ")";
    }
    else
    {
        s << "Group " << obstacle.group_id;
    }
    box.ns = s.str();

    box.header.stamp = ros::Time::now();
    box.type         = visualization_msgs::Marker::CUBE;

    Eigen::Matrix4d T;
    T.noalias() = obstacle.state.getPose(timeStep) * obstacle.bounding_box.T;

    // Calculate origin of the marker
    Eigen::Vector3d c;
    c << obstacle.bounding_box.length_x / 2.0, obstacle.bounding_box.length_y / 2.0, 0.0;
    c = T.block<3, 3>(0, 0) * c;

    box.pose.position.x = T(0, 3) + c(0);
    box.pose.position.y = T(1, 3) + c(1);
    box.pose.position.z = T(2, 3) + c(2);

    geometry_msgs::Pose p;
    robot_misc::Common::poseEigenToMsg(T, p);
    box.pose.orientation = p.orientation;

    // first edge already fits
    std::vector<visualization_msgs::Marker> edge1 = createCylinderMarker(obstacle, color);

    // second edge has to be rotated
    Obstacle tmp_edge_2              = obstacle;
    tmp_edge_2.bounding_box.T        = robot_misc::Common::rotz(M_PI / 2.0);
    tmp_edge_2.bounding_box.length_x = obstacle.bounding_box.length_y;

    std::vector<visualization_msgs::Marker> edge2 = createCylinderMarker(tmp_edge_2, color);

    // third edge has to be shifted in y direction
    Obstacle tmp_edge_3             = obstacle;
    tmp_edge_3.bounding_box.T(1, 3) = obstacle.bounding_box.length_y;

    std::vector<visualization_msgs::Marker> edge3 = createCylinderMarker(tmp_edge_3, color);

    // fourth edge has to be rotated and shifted
    Obstacle tmp_edge_4              = obstacle;
    Eigen::Matrix4d t                = robot_misc::Common::rotz(M_PI / 2.0);
    t(0, 3)                          = obstacle.bounding_box.length_x;
    tmp_edge_4.bounding_box.T        = t;
    tmp_edge_4.bounding_box.length_x = obstacle.bounding_box.length_y;

    std::vector<visualization_msgs::Marker> edge4 = createCylinderMarker(tmp_edge_4, color);

    return {edge1[0], edge1[1], edge1[2], edge2[1], edge2[2], edge3[1], edge3[2], edge4[1], box};
}

std::vector<visualization_msgs::Marker> ObstaclePublisher::createExtrudedCubeMarker(const Obstacle& obstacle, const Color& color,
                                                                                    const double& timeStep) const
{
    visualization_msgs::Marker top;

    top.header.frame_id = "world";
    top.action          = visualization_msgs::Marker::ADD;
    top.lifetime        = ros::Duration(0.0);

    top.scale.x = obstacle.bounding_box.length_x;
    top.scale.y = obstacle.bounding_box.length_y;
    top.scale.z = 0.01;

    top.color.r = color.r;
    top.color.g = color.g;
    top.color.b = color.b;
    top.color.a = color.a;

    std::ostringstream s;
    if (obstacle.group_id == 0)
    {
        s << obstacle.name << " (" << obstacle.id << ")";
    }
    else
    {
        s << "Group " << obstacle.group_id;
    }
    top.ns = s.str();

    top.header.stamp = ros::Time::now();
    top.type         = visualization_msgs::Marker::CUBE;

    Eigen::Matrix4d T;
    T.noalias() = obstacle.state.getPose(timeStep) * obstacle.bounding_box.T;

    // Calculate origin of the top marker
    Eigen::Vector4d c;
    c << obstacle.bounding_box.length_x / 2.0, obstacle.bounding_box.length_y / 2.0, -top.scale.z / 2.0, 1.0;
    c = T * c;

    top.pose.position.x = c(0);
    top.pose.position.y = c(1);
    top.pose.position.z = c(2);

    geometry_msgs::Pose p;
    robot_misc::Common::poseEigenToMsg(T, p);
    top.pose.orientation = p.orientation;

    // Copy top marker for faces and reset some properties
    visualization_msgs::Marker faces = top;
    faces.type                       = visualization_msgs::Marker::TRIANGLE_LIST;
    faces.scale.x                    = 1.0;
    faces.scale.y                    = 1.0;
    faces.scale.z                    = 1.0;
    robot_misc::Common::poseEigenToMsg(Eigen::Matrix4d::Identity(), faces.pose);

    Eigen::Vector4d v, w, vw;
    v << obstacle.bounding_box.length_x, 0.0, 0.0, 1.0;
    w << 0.0, obstacle.bounding_box.length_y, 0.0, 1.0;
    vw    = v + w;
    vw(3) = 1.0;
    v     = T * v;
    w     = T * w;
    vw    = T * vw;

    geometry_msgs::Point p0, p1, p2, p3, p0g, p1g, p2g, p3g;
    p0.x  = T(0, 3);
    p0.y  = T(1, 3);
    p0.z  = T(2, 3);
    p1.x  = v(0);
    p1.y  = v(1);
    p1.z  = v(2);
    p2.x  = w(0);
    p2.y  = w(1);
    p2.z  = w(2);
    p3.x  = vw(0);
    p3.y  = vw(1);
    p3.z  = vw(2);
    p0g   = p0;
    p0g.z = 0.0;
    p1g   = p1;
    p1g.z = 0.0;
    p2g   = p2;
    p2g.z = 0.0;
    p3g   = p3;
    p3g.z = 0.0;

    faces.points = {p0, p0g, p1g, p0, p1g, p1, p1, p1g, p3g, p1, p3g, p3, p3, p3g, p2g, p3, p2g, p2, p2, p2g, p0g, p2, p0g, p0};

    return {top, faces};
}

visualization_msgs::Marker ObstaclePublisher::createSphereMarker(const Obstacle& obstacle, const Color& color, const double& timeStep) const
{
    visualization_msgs::Marker sphere;

    sphere.header.frame_id = "world";
    sphere.action          = visualization_msgs::Marker::ADD;
    sphere.lifetime        = ros::Duration(0.0);

    sphere.scale.x = 2 * obstacle.bounding_box.getRadius(timeStep);
    sphere.scale.y = 2 * obstacle.bounding_box.getRadius(timeStep);
    sphere.scale.z = 2 * obstacle.bounding_box.getRadius(timeStep);

    sphere.color.r = color.r;
    sphere.color.g = color.g;
    sphere.color.b = color.b;
    sphere.color.a = color.a;

    std::ostringstream s;
    if (obstacle.group_id == 0)
    {
        s << obstacle.name << " (" << obstacle.id << ")";
    }
    else
    {
        s << "Group " << obstacle.group_id;
    }
    sphere.ns = s.str();

    sphere.header.stamp = ros::Time::now();
    sphere.type         = visualization_msgs::Marker::SPHERE;

    // Transform into start point
    Eigen::Matrix4d T;
    T.noalias() = obstacle.state.getPose(timeStep) * obstacle.bounding_box.T;

    //    if (obstacle.name.compare("Head") == 0)
    //    {
    //        std::cout << "Body part: " << obstacle.name << "\n with Pose: \n"
    //                  << obstacle.state.getPose(timeStep) << "\n at time step: " << timeStep << "\n"
    //                  << "and at timestep 0: \n"
    //                  << obstacle.state.getPose() << "\n"
    //                  << std::endl;
    //    }

    sphere.pose.position.x = T(0, 3);
    sphere.pose.position.y = T(1, 3);
    sphere.pose.position.z = T(2, 3);

    sphere.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1).normalize());

    return sphere;
}

std::vector<visualization_msgs::Marker> ObstaclePublisher::createUncertaintySphereMarker(const ObstaclePublisher::Obstacle& obstacle,
                                                                                         const ObstaclePublisher::Color& color,
                                                                                         const double& timeStep) const
{

    std::vector<visualization_msgs::Marker> spheres(obstacle.uncertainty_states.size());

    for (int cnt = 0; cnt < spheres.size(); ++cnt)
    {
        visualization_msgs::Marker sphere;

        sphere.header.frame_id = "world";
        sphere.action          = visualization_msgs::Marker::ADD;
        sphere.lifetime        = ros::Duration(0.0);

        sphere.scale.x = 2 * obstacle.bounding_box.getRadius(timeStep);
        sphere.scale.y = 2 * obstacle.bounding_box.getRadius(timeStep);
        sphere.scale.z = 2 * obstacle.bounding_box.getRadius(timeStep);

        sphere.color.r = color.r;
        sphere.color.g = color.g;
        sphere.color.b = color.b;
        sphere.color.a = color.a;

        std::ostringstream s;
        if (obstacle.group_id == 0)
        {
            s << obstacle.name << " (" << obstacle.id << ")";
        }
        else
        {
            s << "Group " << obstacle.group_id;
        }
        sphere.ns = s.str();

        sphere.header.stamp = ros::Time::now();
        sphere.type         = visualization_msgs::Marker::SPHERE;

        // Transform into start point
        Eigen::Matrix4d T;
        T.noalias() = obstacle.uncertainty_states.at(cnt).getPose(timeStep) * obstacle.bounding_box.T;

        //    if (obstacle.name.compare("Head") == 0)
        //    {
        //        std::cout << "Body part: " << obstacle.name << "\n with Pose: \n"
        //                  << obstacle.state.getPose(timeStep) << "\n at time step: " << timeStep << "\n"
        //                  << "and at timestep 0: \n"
        //                  << obstacle.state.getPose() << "\n"
        //                  << std::endl;
        //    }

        sphere.pose.position.x = T(0, 3);
        sphere.pose.position.y = T(1, 3);
        sphere.pose.position.z = T(2, 3);

        sphere.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1).normalize());

        spheres[cnt] = sphere;
    }

    return spheres;
}

// visualization_msgs::Marker ObstaclePublisher::createSphereMarker(const Obstacle& obstacle, const Color& color, const double& timeStep,
//                                                                 const double& lifetime) const
//{
//    visualization_msgs::Marker sphere = createSphereMarker(obstacle, color, timeStep);
//    sphere.header.stamp               = ros::Time::now();
//    sphere.lifetime                   = ros::Duration(lifetime);
//    return sphere;
//}

void ObstaclePublisher::spawnGazeboModel(const std::stringstream& name, const std::string& description, ros::ServiceClient& gazebo_service_client,
                                         const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose)
{
    gazebo_msgs::SpawnModel service;

    // Fill msg
    robot_misc::Common::poseEigenToMsg(pose, service.request.initial_pose);

    service.request.model_name      = name.str();
    service.request.model_xml       = description;
    service.request.reference_frame = "world";

    if (!gazebo_service_client.call(service))
    {
        ROS_WARN("Failed to spawn model.");
    }
}

void ObstaclePublisher::spawnStaticObstacles(ros::ServiceClient& gazebo_service_client, const std::map<int, Obstacle>& static_obstacles)
{
    std::string type  = "";
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    // loop through static obstacles
    for (auto const& obs : static_obstacles)
    {
        const Obstacle& obstacle = obs.second;

        // Skip spawning if not requested
        if (!obstacle.gazebo) continue;

        if (obstacle.bounding_box.type == BoundingBoxType::CYLINDER)
        {
            type = "cylinder";

            // Wrap urdf cylinder convention
            T.noalias() = obstacle.state.getPose() * obstacle.bounding_box.T * robot_misc::Common::roty(M_PI / 2.0);
        }
        else if (obstacle.bounding_box.type == BoundingBoxType::SPHERE)
        {
            type        = "sphere";
            T.noalias() = obstacle.state.getPose() * obstacle.bounding_box.T;
        }
        else if (obstacle.bounding_box.type == BoundingBoxType::BOX)
        {
            type        = "box";
            T.noalias() = obstacle.state.getPose() * obstacle.bounding_box.T;
        }
        else if (obstacle.bounding_box.type == BoundingBoxType::EBOX)
        {
            type        = "ebox";
            T.noalias() = obstacle.state.getPose() * obstacle.bounding_box.T;
        }
        else
        {
            ROS_ERROR("Obstacle type not supported. Skipping obstacle...");
            continue;
        }

        std::stringstream command;
        command << "rosrun xacro xacro --inorder $(rospack find mhp_robot)/config/obstacle_spawner/static_obstacle.urdf.xacro type:='" << type
                << "' id:=" << obstacle.id << " radius:=" << obstacle.bounding_box.radius << " length_x:=" << obstacle.bounding_box.length_x
                << " length_y:=" << obstacle.bounding_box.length_y;

        // run command and get converted urdf
        std::string result = exec(command.str().c_str());

        // Model name
        std::stringstream name;
        name << "static_obstacle_" << obstacle.id;

        // spawn bounding box via urdf in gazebo
        spawnGazeboModel(name, result, gazebo_service_client, T);
    }
}

void ObstaclePublisher::spawnDynamicObstacles(ros::ServiceClient& gazebo_service_client, const std::map<int, Obstacle>& dynamic_obstacles)
{
    std::string type  = "";
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    // loop through dynamic obstacles
    for (auto const& obs : dynamic_obstacles)
    {
        const Obstacle& obstacle = obs.second;

        // Skip spawning if not requested
        if (!obstacle.gazebo) continue;

        if (obstacle.bounding_box.type == BoundingBoxType::CYLINDER)
        {
            type = "cylinder";

            // Wrap urdf cylinder convention
            T.noalias() = obstacle.state.getPose() * obstacle.bounding_box.T * robot_misc::Common::roty(M_PI / 2.0);
        }
        else if (obstacle.bounding_box.type == BoundingBoxType::SPHERE)
        {
            type        = "sphere";
            T.noalias() = obstacle.state.getPose() * obstacle.bounding_box.T;
        }
        else if (obstacle.bounding_box.type == BoundingBoxType::BOX)
        {
            type        = "box";
            T.noalias() = obstacle.state.getPose() * obstacle.bounding_box.T;
        }
        else if (obstacle.bounding_box.type == BoundingBoxType::EBOX)
        {
            type        = "ebox";
            T.noalias() = obstacle.state.getPose() * obstacle.bounding_box.T;
        }
        else
        {
            ROS_ERROR("Obstacle type not supported. Skipping obstacle...");
            continue;
        }

        std::stringstream command;
        command << "rosrun xacro xacro --inorder $(rospack find mhp_robot)/config/obstacle_spawner/dynamic_obstacle.urdf.xacro type:='" << type
                << "' id:=" << obstacle.id << " radius:=" << obstacle.bounding_box.radius << " length_x:=" << obstacle.bounding_box.length_x
                << " length_y:=" << obstacle.bounding_box.length_y;

        // run command and get converted urdf
        std::string result = exec(command.str().c_str());

        // Model name
        std::stringstream name;
        name << "dynamic_obstacle_" << obstacle.id;

        // spawn bounding box via urdf in gazebo
        spawnGazeboModel(name, result, gazebo_service_client, T);
    }
}

void ObstaclePublisher::spawnHumans(ros::ServiceClient& gazebo_service_client, const std::map<int, ObstaclePublisher::Human>& humans)
{
    std::string type  = "";
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    // loop through humans
    for (auto const& hum : humans)
    {
        const Human& human = hum.second;

        // Skip spawning if not requested
        if (!human._gazebo) continue;

        for (auto const& bdy : human._body_parts)
        {
            const Obstacle& body = bdy.second;

            if (body.bounding_box.type == BoundingBoxType::CYLINDER)
            {
                type = "cylinder";

                // Wrap urdf cylinder convention
                T.noalias() = body.state.getPose() * body.bounding_box.T * robot_misc::Common::roty(M_PI / 2.0);
            }
            else if (body.bounding_box.type == BoundingBoxType::SPHERE)
            {
                type        = "sphere";
                T.noalias() = body.state.getPose() * body.bounding_box.T;
            }
            else if (body.bounding_box.type == BoundingBoxType::BOX)
            {
                type        = "box";
                T.noalias() = body.state.getPose() * body.bounding_box.T;
            }
            else if (body.bounding_box.type == BoundingBoxType::EBOX)
            {
                type        = "ebox";
                T.noalias() = body.state.getPose() * body.bounding_box.T;
            }
            else
            {
                ROS_ERROR("Body type not supported. Skipping body...");
                continue;
            }

            std::stringstream command;
            command << "rosrun xacro xacro --inorder $(rospack find mhp_robot)/config/obstacle_spawner/dynamic_obstacle.urdf.xacro type:='" << type
                    << "' id:=" << body.id << " radius:=" << body.bounding_box.radius << " length_x:=" << body.bounding_box.length_x
                    << " length_y:=" << body.bounding_box.length_y;

            // run command and get converted urdf
            std::string result = exec(command.str().c_str());

            // Model name
            std::stringstream name;
            name << "human_" << human._id << "_" << body.name;

            // spawn bounding box via urdf in gazebo
            spawnGazeboModel(name, result, gazebo_service_client, T);
        }
    }
}

void ObstaclePublisher::spawnPlanes(ros::ServiceClient& gazebo_service_client, const std::map<int, ObstaclePublisher::Plane>& planes)
{
    // loop through planes
    for (auto const& pl : planes)
    {
        const Plane& plane = pl.second;

        // Skip spawning if not requested
        if (!plane.gazebo) continue;

        std::stringstream command;
        command << "rosrun xacro xacro --inorder $(rospack find mhp_robot)/config/obstacle_spawner/plane.urdf.xacro id:=" << plane.id;

        // run command and get converted urdf
        std::string result = exec(command.str().c_str());

        // Model name
        std::stringstream name;
        name << "plane_" << plane.id;

        // spawn bounding box via urdf in gazebo
        spawnGazeboModel(name, result, gazebo_service_client, robot_misc::Common::poseFromPlane(plane.q, plane.n));
    }
}

std::string ObstaclePublisher::exec(const char* cmd) const
{
    char buffer[128];
    std::string result = "";
    FILE* pipe         = popen(cmd, "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try
    {
        while (!feof(pipe))
        {
            if (fgets(buffer, 128, pipe) != NULL) result += buffer;
        }
    }
    catch (...)
    {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    return result;
}

}  // namespace robot_obstacle
}  // namespace mhp_robot
