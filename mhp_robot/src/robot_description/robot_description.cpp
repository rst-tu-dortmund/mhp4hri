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

#include <mhp_robot/robot_description/robot_description.h>
#include <mhp_robot/robot_misc/common.h>
#include <ros/io.h>
#include <iostream>

namespace mhp_robot {
namespace robot_description {

RobotDescription::RobotDescription() { setDefaultPlanes(_ground_plane, _roof_plane); }

RobotDescription::RobotDescription(const std::string& urdf_param, const std::string& first_frame, const std::string& last_frame)
    : _urdf_param(urdf_param), _first_frame(first_frame), _last_frame(last_frame)
{
    if (urdf_param.empty())
    {
        ROS_ERROR("RobotDescription: Invalid parameters.");
    }
    parseURDF(urdf_param, _first_frame, _last_frame);
}

RobotDescription::UPtr RobotDescription::createUniqueInstance() const
{
    return std::make_unique<RobotDescription>(_urdf_param, _first_frame, _last_frame);
}

RobotDescription::Ptr RobotDescription::createSharedInstance() const
{
    return std::make_shared<RobotDescription>(_urdf_param, _first_frame, _last_frame);
}

bool RobotDescription::isInitialized() const { return _initialized; }

const std::string& RobotDescription::getURDFParam() const { return _urdf_param; }

const std::string& RobotDescription::getFirstFrame() const { return _first_frame; }

const std::string& RobotDescription::getLastFrame() const { return _last_frame; }

const robot_misc::Plane& RobotDescription::getGroundPlane() const { return _ground_plane; }

const robot_misc::Plane& RobotDescription::getRoofPlane() const { return _roof_plane; }

const std::vector<robot_misc::Segment>& RobotDescription::getKinematicStructure() const
{
    if (!_initialized) ROS_WARN("Kinematic is not initialized!");
    return _segment_structs;
}

const urdf::Model& RobotDescription::getURDFModel() const
{
    if (!_initialized) ROS_WARN("Kinematic is not initialized!");
    return _urdf_model;
}

bool RobotDescription::parseURDF(const std::string& param, const std::string& first_frame, const std::string& last_frame)
{
    if (!_urdf_model.initParam(param))
    {
        ROS_ERROR("Could not load urdf from parameter server.");
        return false;
    }

    _first_frame = first_frame;
    _last_frame  = last_frame;

    urdf::Link last_link;

    if (!hasFrame(_first_frame, *_urdf_model.getRoot(), _first_link)) _first_link = *_urdf_model.getRoot();
    if (!hasFrame(_last_frame, _first_link, last_link, true))
    {
        ROS_ERROR("Last Frame could not be found.");
        return false;
    }

    // reverse vector because of recursion
    std::reverse(_segment_structs.begin(), _segment_structs.end());

    if (!checkParameters())
    {
        ROS_ERROR("RobotDescription: Invalid parameters.");
        return false;
    }

    setDefaultPlanes(_ground_plane, _roof_plane);

    // Extract Ground and Roof
    if (!extractGroundPlane(*_urdf_model.getRoot(), _ground_plane)) ROS_WARN("No ground plane found. Using default ground.");
    if (!extractRoofPlane(*_urdf_model.getRoot(), _roof_plane)) ROS_WARN("No roof plane found. Using default roof.");

    _initialized = true;
    return true;
}

bool RobotDescription::checkParameters() const { return true; }

robot_misc::Segment RobotDescription::createSegment(const urdf::Link& link) const
{
    robot_misc::Segment segment;
    segment.bounding_box = createBoundingBox(link);
    segment.joint        = createJoint(*link.parent_joint);
    segment.name         = link.name;

    return segment;
}

robot_misc::BoundingBox RobotDescription::createBoundingBox(const urdf::Link& link) const
{
    robot_misc::BoundingBox bounding_box;

    if (link.collision_array.size() == 0)
    {
        bounding_box.active = false;
        return bounding_box;
    }

    // check for cylinder first
    bool cylinder_found = false;
    for (auto& collision : link.collision_array)
    {
        if (collision->geometry->type == urdf::Geometry::CYLINDER)
        {
            const urdf::Cylinder& cylinder = *urdf::static_pointer_cast<urdf::Cylinder>(collision->geometry);

            // type
            bounding_box.type = robot_misc::BoundingBoxType::CYLINDER;

            // dimensions
            bounding_box.length_x = cylinder.length;
            bounding_box.radius   = cylinder.radius;

            // orientation and position
            double x, y, z, w;
            collision->origin.rotation.getQuaternion(x, y, z, w);
            Eigen::Quaterniond q;
            q.x() = x;
            q.y() = y;
            q.z() = z;
            q.w() = w;

            Eigen::Matrix4d T, Tbb;
            Tbb = Eigen::Matrix4d::Identity();
            T   = Eigen::Matrix4d::Identity();

            // Transform into origin of urdf cylinder
            T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
            T.block<3, 1>(0, 3) << collision->origin.position.x, collision->origin.position.y, collision->origin.position.z;

            // Relative transform into start point of the urdf cylinder
            Tbb(2, 3) = -cylinder.length / 2.0;

            // Final transform from world to start point, with x axis aligned with z axis
            bounding_box.T.noalias() = T * Tbb * robot_misc::Common::roty(-M_PI / 2.0);

            cylinder_found = true;
            break;  // only support one cylinder per link
        }
    }

    // check for spheres only if there were no cylinder
    bool sphere_found = false;
    if (!cylinder_found)
    {
        for (auto& collision : link.collision_array)
        {
            if (collision->geometry->type == urdf::Geometry::SPHERE)
            {
                const urdf::Sphere& sphere = *urdf::static_pointer_cast<urdf::Sphere>(collision->geometry);

                // type
                bounding_box.type = robot_misc::BoundingBoxType::SPHERE;

                // dimensions
                bounding_box.radius = sphere.radius;

                // orientation and position
                double x, y, z, w;
                collision->origin.rotation.getQuaternion(x, y, z, w);
                Eigen::Quaterniond q;
                q.x() = x;
                q.y() = y;
                q.z() = z;
                q.w() = w;

                bounding_box.T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
                bounding_box.T.block<3, 1>(0, 3) << collision->origin.position.x, collision->origin.position.y, collision->origin.position.z;

                sphere_found = true;

                break;  // only support one sphere
            }
        }
    }
    if (!cylinder_found && !sphere_found)
    {
        ROS_WARN("Collision geometry type not supported. Only spehres and cylinders suppoerted. Collisions will be ignored!");
        bounding_box.active = false;
        return bounding_box;
    }

    bounding_box.active = true;

    return bounding_box;
}

robot_misc::Joint RobotDescription::createJoint(const urdf::Joint& joint) const
{
    robot_misc::Joint jt;

    // name
    jt.name = joint.name;

    // limits
    if (joint.limits)
    {
        jt.min      = joint.limits->lower;
        jt.max      = joint.limits->upper;
        jt.effort   = joint.limits->effort;
        jt.velocity = joint.limits->velocity;
    }

    // fixed joint
    jt.fixed = joint.type == urdf::Joint::REVOLUTE ? false : true;

    // axis of roation
    if (joint.axis.x)
    {
        jt.axis[0] = true;
    }
    else if (joint.axis.y)
    {
        jt.axis[1] = true;
    }
    else if (joint.axis.z)
    {
        jt.axis[2] = true;
    }
    else
    {
        if (!jt.fixed) ROS_ERROR_STREAM("Axis of rotation not specified for " << jt.name);
    }

    // transformation
    jt.tx = joint.parent_to_joint_origin_transform.position.x;
    jt.ty = joint.parent_to_joint_origin_transform.position.y;
    jt.tz = joint.parent_to_joint_origin_transform.position.z;

    // orientation
    Eigen::Quaterniond quat(joint.parent_to_joint_origin_transform.rotation.w, joint.parent_to_joint_origin_transform.rotation.x,
                            joint.parent_to_joint_origin_transform.rotation.y, joint.parent_to_joint_origin_transform.rotation.z);

    Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);

    jt.Rx = euler(0);
    jt.Ry = euler(1);
    jt.Rz = euler(2);

    return jt;
}

bool RobotDescription::hasFrame(const std::string& frame, const urdf::Link& target_link, urdf::Link& link, bool construct)
{
    if (target_link.name.compare(frame) == 0)
    {
        // link found
        link = target_link;

        if (construct && (&target_link != &_first_link))
        {
            _segment_structs.push_back(createSegment(target_link));
        }

        return true;
    }
    else
    {
        // for all child joints
        for (auto const& joint : target_link.child_joints)
        {
            if (hasFrame(frame, *_urdf_model.links_[joint->child_link_name], link, construct))
            {
                if (construct && (&target_link != &_first_link))
                {
                    _segment_structs.push_back(createSegment(target_link));
                }
                return true;
            }
        }
    }
    return false;
}

bool RobotDescription::extractGroundPlane(const urdf::Link& world_link, robot_misc::Plane& ground_plane) const
{
    // Look for ground link in all child joints
    for (auto const& joint : world_link.child_joints)
    {
        const urdf::Link& link = *_urdf_model.links_.at(joint->child_link_name);

        if (link.name.compare("ground") == 0)
        {
            // ground link found
            if (joint->type != urdf::Joint::FIXED)
            {
                ROS_ERROR("Ground plane must be static!");
                break;
            }

            // transformation
            ground_plane.q << joint->parent_to_joint_origin_transform.position.x, joint->parent_to_joint_origin_transform.position.y,
                joint->parent_to_joint_origin_transform.position.z;

            // orientation
            double x, y, z, w;
            joint->parent_to_joint_origin_transform.rotation.getQuaternion(x, y, z, w);

            Eigen::Quaterniond q(w, x, y, z);
            ground_plane.n = q.toRotationMatrix().col(2);  // Use z-axis in world coordinates

            return true;
        }
    }

    return false;
}

bool RobotDescription::extractRoofPlane(const urdf::Link& world_link, robot_misc::Plane& roof_plane) const
{
    // Look for ground link in all child joints
    for (auto const& joint : world_link.child_joints)
    {
        const urdf::Link& link = *_urdf_model.links_.at(joint->child_link_name);

        if (link.name.compare("roof") == 0)
        {
            // roof link found
            if (joint->type != urdf::Joint::FIXED)
            {
                ROS_ERROR("Roof plane must be static!");
                break;
            }

            // transformation
            roof_plane.q << joint->parent_to_joint_origin_transform.position.x, joint->parent_to_joint_origin_transform.position.y,
                joint->parent_to_joint_origin_transform.position.z;

            // orientation
            double x, y, z, w;
            joint->parent_to_joint_origin_transform.rotation.getQuaternion(x, y, z, w);

            Eigen::Quaterniond q(w, x, y, z);
            roof_plane.n = q.toRotationMatrix().col(2);  // Use z-axis in world coordinates

            return true;
        }
    }

    return false;
}

void RobotDescription::setDefaultPlanes(robot_misc::Plane& ground_plane, robot_misc::Plane& roof_plane)
{
    // Set ground and roof defaults
    ground_plane.q << 0.0, 0.0, 0.0;
    roof_plane.q << 0.0, 0.0, 1.8;
    roof_plane.n << 0.0, 0.0, -1.0;
}

}  // namespace robot_description
}  // namespace mhp_robot
