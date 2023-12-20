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
#include <mhp_robot/robot_collision/robot_collision_marker_publisher.h>

namespace mhp_robot {
namespace robot_collision {

RobotCollisionMarkerPublisher::RobotCollisionMarkerPublisher(const std::string& joint_state_topic, const std::string& obstacle_topic,
                                                             RobotCollision::UPtr robot_collision, RobotUtility::UPtr robot_utility)
    : _robot_collision(std::move(robot_collision)),
      _robot_utility(std::move(robot_utility)),
      _joint_state_topic(joint_state_topic),
      _obstacle_topic(obstacle_topic)
{
    _joint_states = std::vector<double>(_robot_utility->getJointsCount(), 0.0);

    ros::NodeHandle n("~");

    // Subscriber
    _joint_state_subscriber = n.subscribe(_joint_state_topic, 1, &RobotCollisionMarkerPublisher::jointStateCallback, this);
    _obstacle_subscriber    = n.subscribe(_obstacle_topic, 10, &RobotCollisionMarkerPublisher::obstacleCallback, this);

    // Publisher
    _line_marker_pub  = n.advertise<visualization_msgs::Marker>("collision_distance_marker", 1000);
    _point_marker_pub = n.advertise<visualization_msgs::Marker>("collision_point_marker", 1000);
    _distance_pub     = n.advertise<mhp_robot::MsgDistances>("minimum_distances", 1000);

    // Init distances
    _self_collision_distance   = std::numeric_limits<double>::max();
    _static_obstacle_distance  = std::numeric_limits<double>::max();
    _dynamic_obstacle_distance = std::numeric_limits<double>::max();
    _ground_distance           = std::numeric_limits<double>::max();
    _roof_distance             = std::numeric_limits<double>::max();
    _plane_distance            = std::numeric_limits<double>::max();
    _human_distances           = std::vector<double>(Human::_body_parts_size, std::numeric_limits<double>::max());
    _human_distances_segments  = std::vector<double>(Human::_body_parts_size, std::numeric_limits<double>::max());
}

void RobotCollisionMarkerPublisher::configure(bool self_collision, bool plane_collision, bool ground_collision, bool roof_collision,
                                              bool static_obstacle_collision, bool dynamic_obstacle_collision, bool human_collision,
                                              bool publish_distances)
{
    _self_collision             = self_collision;
    _plane_collision            = plane_collision;
    _ground_collision           = ground_collision;
    _roof_collision             = roof_collision;
    _static_obstacle_collision  = static_obstacle_collision;
    _dynamic_obstacle_collision = dynamic_obstacle_collision;
    _human_collision            = human_collision;
    _publish_distances          = publish_distances;
}

void RobotCollisionMarkerPublisher::publish()
{
    ros::Rate loop(30);

    while (ros::ok())
    {
        if (_self_collision) updateSelfCollisionMarker();
        if (_ground_collision) updateGroundCollisionMarker();
        if (_roof_collision) updateRoofCollisionMarker();
        if (_plane_collision) updatePlaneCollisionMarker();
        if (_static_obstacle_collision) updateStaticObstacleCollisionMarker();
        if (_dynamic_obstacle_collision) updateDynamicObstacleCollisionMarker();
        if (_human_collision) updateHumanCollisionMarker();
        if (_publish_distances) publishDistances();

        ros::spinOnce();
        loop.sleep();
    }
}

void RobotCollisionMarkerPublisher::obstacleCallback(const MsgObstacleListConstPtr& msg)
{
    _obstacle_manager._mutex.lock();
    Common::parseObstacleMsg(msg, _obstacle_manager._static_obstacles, _obstacle_manager._dynamic_obstacles, _obstacle_manager._humans,
                             _obstacle_manager._utility_objects, _obstacle_manager._planes);
    _obstacle_manager._mutex.unlock();
}

void RobotCollisionMarkerPublisher::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    if (_first_joint_state)
    {
        if (!_robot_utility->initJointMapping(msg->name))
        {
            return;
        }
        _first_joint_state = false;
        _robot_utility->parseJointStates(_joint_states, msg->position);
    }
    else
    {
        _robot_utility->parseJointStates(_joint_states, msg->position);
    }
}

void RobotCollisionMarkerPublisher::publishMarker(const Eigen::Ref<const Eigen::Vector3d>& start, const Eigen::Ref<const Eigen::Vector3d>& end,
                                                  double distance, const robot_misc::Color& color, const std::string& label, int id)
{
    visualization_msgs::Marker start_point, end_point, line, distance_text;

    start_point.header.frame_id = "/world";
    start_point.action          = visualization_msgs::Marker::ADD;
    start_point.lifetime        = ros::Duration(0.1);

    start_point.scale.x = 0.05;
    start_point.scale.y = 0.05;
    start_point.scale.z = 0.05;

    start_point.color.r = color.r;
    start_point.color.g = color.g;
    start_point.color.b = color.b;
    start_point.color.a = color.a;

    start_point.ns = label;

    end_point     = start_point;
    line          = start_point;
    distance_text = start_point;

    start_point.header.stamp       = ros::Time::now();
    start_point.type               = visualization_msgs::Marker::SPHERE;
    start_point.id                 = id * _visualization_id_spacing;
    start_point.pose.position.x    = start(0);
    start_point.pose.position.y    = start(1);
    start_point.pose.position.z    = start(2);
    start_point.pose.orientation.w = 1.0;
    start_point.pose.orientation.x = 0.0;
    start_point.pose.orientation.y = 0.0;
    start_point.pose.orientation.z = 0.0;

    end_point.header.stamp       = ros::Time::now();
    end_point.type               = visualization_msgs::Marker::SPHERE;
    end_point.id                 = id * _visualization_id_spacing + 1;
    end_point.pose.position.x    = end(0);
    end_point.pose.position.y    = end(1);
    end_point.pose.position.z    = end(2);
    end_point.pose.orientation.w = 1.0;
    end_point.pose.orientation.x = 0.0;
    end_point.pose.orientation.y = 0.0;
    end_point.pose.orientation.z = 0.0;

    line.header.stamp = ros::Time::now();
    line.type         = visualization_msgs::Marker::LINE_LIST;
    line.id           = id * _visualization_id_spacing + 2;
    line.points.push_back(start_point.pose.position);
    line.points.push_back(end_point.pose.position);
    line.scale.x            = 0.01;
    line.scale.y            = 0.0;
    line.scale.z            = 0.0;
    line.pose.orientation.w = 1.0;
    line.pose.orientation.x = 0.0;
    line.pose.orientation.y = 0.0;
    line.pose.orientation.z = 0.0;

    distance_text.header.stamp = ros::Time::now();
    distance_text.scale.x      = 0.0;
    distance_text.scale.y      = 0.0;
    distance_text.type         = visualization_msgs::Marker::TEXT_VIEW_FACING;
    distance_text.id           = id * _visualization_id_spacing + 3;

    std::ostringstream strs;
    strs << distance;
    distance_text.text = strs.str();

    geometry_msgs::Point text_point;
    text_point.x = start_point.pose.position.x / 2.0 + end_point.pose.position.x / 2.0;
    text_point.y = start_point.pose.position.y / 2.0 + end_point.pose.position.y / 2.0;
    text_point.z = start_point.pose.position.z / 2.0 + end_point.pose.position.z / 2.0;

    distance_text.pose.position = text_point;

    _point_marker_pub.publish(start_point);
    _point_marker_pub.publish(end_point);
    _line_marker_pub.publish(line);
    _line_marker_pub.publish(distance_text);
}

void RobotCollisionMarkerPublisher::updateSelfCollisionMarker()
{
    Eigen::VectorXd joint_state = Eigen::Map<Eigen::VectorXd>(_joint_states.data(), _joint_states.size());
    Eigen::Matrix<double, 3, 1> obstacle_start_point;
    Eigen::Matrix<double, 3, 1> obstacle_end_point;

    _robot_collision->setJointState(joint_state);

    double min_d = _robot_collision->getMinSelfCollisionDistance(obstacle_start_point, obstacle_end_point);

    // Publish
    publishMarker(obstacle_start_point, obstacle_end_point, min_d, robot_misc::Color(0, 1, 0), "Self Collision", 0);

    // Update minimum distances
    _self_collision_distance = min_d;
}

void RobotCollisionMarkerPublisher::updateGroundCollisionMarker()
{
    Eigen::VectorXd joint_state = Eigen::Map<Eigen::VectorXd>(_joint_states.data(), _joint_states.size());
    Eigen::Matrix<double, 3, 1> obstacle_start_point;
    Eigen::Matrix<double, 3, 1> obstacle_end_point;

    _robot_collision->setJointState(joint_state);

    double min_d = _robot_collision->getMinGroundCollisionDistance(obstacle_start_point, obstacle_end_point);

    // Publish
    publishMarker(obstacle_start_point, obstacle_end_point, min_d, robot_misc::Color(0, 0, 1), "Ground Collision", 0);

    // Update minimum distances
    _ground_distance = min_d;
}

void RobotCollisionMarkerPublisher::updateRoofCollisionMarker()
{
    Eigen::VectorXd joint_state = Eigen::Map<Eigen::VectorXd>(_joint_states.data(), _joint_states.size());
    Eigen::Matrix<double, 3, 1> obstacle_start_point;
    Eigen::Matrix<double, 3, 1> obstacle_end_point;

    _robot_collision->setJointState(joint_state);

    double min_d = _robot_collision->getMinRoofCollisionDistance(obstacle_start_point, obstacle_end_point);

    // Publish
    publishMarker(obstacle_start_point, obstacle_end_point, min_d, robot_misc::Color(0, 0, 1), "Roof Collision", 0);

    // Update minimum distances
    _roof_distance = min_d;
}

void RobotCollisionMarkerPublisher::updatePlaneCollisionMarker()
{
    Eigen::VectorXd joint_state = Eigen::Map<Eigen::VectorXd>(_joint_states.data(), _joint_states.size());
    Eigen::Matrix<double, 3, 1> obstacle_start_point;
    Eigen::Matrix<double, 3, 1> obstacle_end_point;

    _robot_collision->setJointState(joint_state);

    _obstacle_manager._mutex.lock();
    std::vector<Plane> planes = _obstacle_manager._planes;
    _obstacle_manager._mutex.unlock();

    _plane_distance = std::numeric_limits<double>::max();

    for (int i = 0; i < planes.size(); ++i)
    {
        const Plane& plane = planes[i];

        double min_d = _robot_collision->getMinPlaneCollisionDistance(plane, obstacle_start_point, obstacle_end_point);

        std::string label;
        if (plane.group_id == 0)
        {
            label = plane.name + " (" + std::to_string(plane.id) + ")";
        }
        else
        {
            label = "Group " + std::to_string(plane.group_id);
        }

        // Publish
        publishMarker(obstacle_start_point, obstacle_end_point, min_d, robot_misc::Color(0, 0, 1), label, i);

        // Update minimum distances
        if (min_d < _plane_distance)
        {
            _plane_distance = min_d;
        }
    }
}

void RobotCollisionMarkerPublisher::updateStaticObstacleCollisionMarker()
{
    Eigen::VectorXd joint_state = Eigen::Map<Eigen::VectorXd>(_joint_states.data(), _joint_states.size());
    Eigen::Matrix<double, 3, 1> obstacle_start_point;
    Eigen::Matrix<double, 3, 1> obstacle_end_point;

    _robot_collision->setJointState(joint_state);

    _obstacle_manager._mutex.lock();
    std::vector<Obstacle> static_obstacles = _obstacle_manager._static_obstacles;
    _obstacle_manager._mutex.unlock();

    _static_obstacle_distance = std::numeric_limits<double>::max();

    for (int i = 0; i < static_obstacles.size(); ++i)
    {
        const Obstacle& obstacle = static_obstacles[i];

        double min_d = _robot_collision->getMinObstacleDistance(obstacle, obstacle_start_point, obstacle_end_point);

        std::string label;
        if (obstacle.group_id == 0)
        {
            label = obstacle.name + " (" + std::to_string(obstacle.id) + ")";
        }
        else
        {
            label = "Static Group " + std::to_string(obstacle.group_id);
        }

        // Publish
        publishMarker(obstacle_start_point, obstacle_end_point, min_d, robot_misc::Color(1, 0, 0), label, i);

        // Update minimum distances
        if (min_d < _static_obstacle_distance)
        {
            _static_obstacle_distance = min_d;
        }
    }
}

void RobotCollisionMarkerPublisher::updateDynamicObstacleCollisionMarker()
{
    Eigen::VectorXd joint_state = Eigen::Map<Eigen::VectorXd>(_joint_states.data(), _joint_states.size());
    Eigen::Matrix<double, 3, 1> obstacle_start_point;
    Eigen::Matrix<double, 3, 1> obstacle_end_point;

    _robot_collision->setJointState(joint_state);

    _obstacle_manager._mutex.lock();
    std::vector<Obstacle> dynamic_obstacles = _obstacle_manager._dynamic_obstacles;
    _obstacle_manager._mutex.unlock();

    _dynamic_obstacle_distance = std::numeric_limits<double>::max();

    for (int i = 0; i < dynamic_obstacles.size(); ++i)
    {
        const Obstacle& obstacle = dynamic_obstacles[i];

        double min_d = _robot_collision->getMinObstacleDistance(obstacle, obstacle_start_point, obstacle_end_point);

        std::string label;
        if (obstacle.group_id == 0)
        {
            label = obstacle.name + " (" + std::to_string(obstacle.id) + ")";
        }
        else
        {
            label = "Dynamic Group " + std::to_string(obstacle.group_id);
        }

        // Publish
        publishMarker(obstacle_start_point, obstacle_end_point, min_d, robot_misc::Color(1, 0, 0), label, i);

        // Update minimum distances
        if (min_d < _dynamic_obstacle_distance)
        {
            _dynamic_obstacle_distance = min_d;
        }
    }
}

void RobotCollisionMarkerPublisher::updateHumanCollisionMarker()
{
    Eigen::VectorXd d                     = Eigen::VectorXd::Zero(Human::_body_parts_size);
    Eigen::VectorXd joint_state           = Eigen::Map<Eigen::VectorXd>(_joint_states.data(), _joint_states.size());
    Eigen::MatrixXd obstacle_start_points = Eigen::MatrixXd::Zero(3, Human::_body_parts_size);
    Eigen::MatrixXd obstacle_end_points   = Eigen::MatrixXd::Zero(3, Human::_body_parts_size);
    std::vector<int> segments(Human::_body_parts_size);

    _robot_collision->setJointState(joint_state);

    _obstacle_manager._mutex.lock();
    std::vector<Human> humans = _obstacle_manager._humans;
    _obstacle_manager._mutex.unlock();

    _human_distances          = std::vector<double>(Human::_body_parts_size, std::numeric_limits<double>::max());
    _human_distances_segments = std::vector<double>(Human::_body_parts_size, std::numeric_limits<double>::max());

    for (int i = 0; i < humans.size(); ++i)
    {
        const Human& human = humans[i];

        _robot_collision->getMinHumanDistance(human, d, obstacle_start_points, obstacle_end_points, segments);

        std::string label = human._name + " (" + std::to_string(human._id) + ")";

        for (int j = 0; j < Human::_body_parts_size; ++j)
        {
            // Publish
            publishMarker(obstacle_start_points.col(j), obstacle_end_points.col(j), d(j), robot_misc::Color(1, 0, 0), label,
                          i * Human::_body_parts_size + j);

            // Update minimum distances
            if (d(j) < _human_distances[j])
            {
                _human_distances[j] = d(j);
                //                std::cout << segments(j) << "\n" << std::endl;
                _human_distances_segments[j] = segments[j];
            }
        }
    }
}

void RobotCollisionMarkerPublisher::publishDistances()
{
    MsgDistances msg;

    msg.self_collision_distance   = _self_collision_distance;
    msg.static_obstacle_distance  = _static_obstacle_distance;
    msg.dynamic_obstacle_distance = _dynamic_obstacle_distance;
    msg.ground_distance           = _ground_distance;
    msg.roof_distance             = _roof_distance;
    msg.plane_distance            = _plane_distance;
    msg.human_distances           = _human_distances;
    msg.human_distances_segments  = _human_distances_segments;
    msg.header.stamp              = ros::Time::now();

    _distance_pub.publish(msg);
}

}  // namespace robot_collision
}  // namespace mhp_robot
