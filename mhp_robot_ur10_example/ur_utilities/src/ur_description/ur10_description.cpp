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
#include <ur_utilities/ur_description/ur10_description.h>

namespace mhp_robot {
namespace robot_description {

UR10Description::UR10Description() { initializeSegmentStructs(); }

void UR10Description::initializeSegmentStructs()
{
    robot_misc::Segment pedestal;
    pedestal.joint.tx     = 0.0;
    pedestal.joint.ty     = 0.0;
    pedestal.joint.tz     = 0.0;
    pedestal.joint.Rx     = 0.0;
    pedestal.joint.Ry     = 0.0;
    pedestal.joint.Rz     = 0.0;
    pedestal.joint.offset = 0;
    pedestal.joint.fixed  = true;
    pedestal.joint.axis   = {false, false, false};

    pedestal.bounding_box.type     = robot_misc::BoundingBoxType::CYLINDER;
    pedestal.bounding_box.length_x = 0.76;
    pedestal.bounding_box.radius   = 0.1;

    pedestal.bounding_box.T = robot_misc::Common::roty(-M_PI / 2.0);

    _segment_structs.push_back(pedestal);

    /* ------------------------------------------ */

    robot_misc::Segment base;
    base.joint.tx     = 0.0;
    base.joint.ty     = 0.0;
    base.joint.tz     = 0.8;
    base.joint.Rx     = 0.0;
    base.joint.Ry     = 0.0;
    base.joint.Rz     = 0.0;
    base.joint.offset = 0;
    base.joint.fixed  = true;
    base.joint.axis   = {false, false, false};

    base.bounding_box.type   = robot_misc::BoundingBoxType::NOTYPE;
    base.bounding_box.active = false;

    _segment_structs.push_back(base);

    /* ------------------------------------------ */

    robot_misc::Segment shoulder;
    shoulder.joint.tx     = 0.0;
    shoulder.joint.ty     = 0.0;
    shoulder.joint.tz     = 0.1273;
    shoulder.joint.Rx     = 0.0;
    shoulder.joint.Ry     = 0.0;
    shoulder.joint.Rz     = 0.0;
    shoulder.joint.offset = 0;
    shoulder.joint.fixed  = false;
    shoulder.joint.axis   = {false, false, true};

    shoulder.bounding_box.type     = robot_misc::BoundingBoxType::SPHERE;
    shoulder.bounding_box.length_x = 0.0;
    shoulder.bounding_box.radius   = 0.11;
    shoulder.bounding_box.T        = Eigen::Matrix<double, 4, 4>::Identity();

    _segment_structs.push_back(shoulder);

    /* ------------------------------------------ */

    robot_misc::Segment upper_arm;
    upper_arm.joint.tx     = 0.0;
    upper_arm.joint.ty     = 0.220941;
    upper_arm.joint.tz     = 0.0;
    upper_arm.joint.Rx     = 0.0;
    upper_arm.joint.Ry     = M_PI / 2.0;
    upper_arm.joint.Rz     = 0.0;
    upper_arm.joint.offset = 0;
    upper_arm.joint.fixed  = false;
    upper_arm.joint.axis   = {false, true, false};

    upper_arm.bounding_box.type     = robot_misc::BoundingBoxType::CYLINDER;
    upper_arm.bounding_box.length_x = 0.6;
    upper_arm.bounding_box.radius   = 0.11;

    Eigen::Matrix<double, 4, 4> T_bbox_upper_arm = robot_misc::Common::roty(-M_PI / 2.0);
    T_bbox_upper_arm(1, 3)                       = -0.05;

    upper_arm.bounding_box.T = T_bbox_upper_arm;

    _segment_structs.push_back(upper_arm);

    /* ------------------------------------------ */

    robot_misc::Segment fore_arm;
    fore_arm.joint.tx     = 0.0;
    fore_arm.joint.ty     = -0.1719;
    fore_arm.joint.tz     = 0.612;
    fore_arm.joint.Rx     = 0.0;
    fore_arm.joint.Ry     = 0.0;
    fore_arm.joint.Rz     = 0.0;
    fore_arm.joint.offset = 0;
    fore_arm.joint.fixed  = false;
    fore_arm.joint.axis   = {false, true, false};

    fore_arm.bounding_box.type     = robot_misc::BoundingBoxType::CYLINDER;
    fore_arm.bounding_box.length_x = 0.56;
    fore_arm.bounding_box.radius   = 0.08;
    fore_arm.bounding_box.T        = robot_misc::Common::roty(-M_PI / 2.0);

    _segment_structs.push_back(fore_arm);

    /* ------------------------------------------ */

    robot_misc::Segment wrist1;
    wrist1.joint.tx     = 0.0;
    wrist1.joint.ty     = 0.0;
    wrist1.joint.tz     = 0.5723;
    wrist1.joint.Rx     = 0.0;
    wrist1.joint.Ry     = M_PI / 2.0;
    wrist1.joint.Rz     = 0.0;
    wrist1.joint.offset = 0;
    wrist1.joint.fixed  = false;
    wrist1.joint.axis   = {false, true, false};

    wrist1.bounding_box.type     = robot_misc::BoundingBoxType::SPHERE;
    wrist1.bounding_box.length_x = 0.0;
    wrist1.bounding_box.radius   = 0.08;

    Eigen::Matrix<double, 4, 4> T_bbox_wrist1 = Eigen::Matrix<double, 4, 4>::Identity();
    T_bbox_wrist1(1, 3)                       = 0.1149;

    wrist1.bounding_box.T = T_bbox_wrist1;

    _segment_structs.push_back(wrist1);

    /* ------------------------------------------ */

    robot_misc::Segment wrist2;
    wrist2.joint.tx     = 0.0;
    wrist2.joint.ty     = 0.1149;
    wrist2.joint.tz     = 0.0;
    wrist2.joint.Rx     = 0.0;
    wrist2.joint.Ry     = 0.0;
    wrist2.joint.Rz     = 0.0;
    wrist2.joint.offset = 0;
    wrist2.joint.fixed  = false;
    wrist2.joint.axis   = {false, false, true};

    wrist2.bounding_box.type     = robot_misc::BoundingBoxType::SPHERE;
    wrist2.bounding_box.length_x = 0.0;
    wrist2.bounding_box.radius   = 0.08;

    Eigen::Matrix<double, 4, 4> T_bbox_wrist2 = Eigen::Matrix<double, 4, 4>::Identity();
    T_bbox_wrist2(2, 3)                       = 0.1157;

    wrist2.bounding_box.T = T_bbox_wrist2;

    _segment_structs.push_back(wrist2);

    /* ------------------------------------------ */

    robot_misc::Segment wrist3;
    wrist3.joint.tx     = 0.0;
    wrist3.joint.ty     = 0.0;
    wrist3.joint.tz     = 0.1157;
    wrist3.joint.Rx     = 0.0;
    wrist3.joint.Ry     = 0.0;
    wrist3.joint.Rz     = 0.0;
    wrist3.joint.offset = 0;
    wrist3.joint.fixed  = false;
    wrist3.joint.axis   = {false, true, false};

    wrist3.bounding_box.type     = robot_misc::BoundingBoxType::SPHERE;
    wrist3.bounding_box.length_x = 0.0;
    wrist3.bounding_box.radius   = 0.1;

    Eigen::Matrix<double, 4, 4> T_bbox_wrist3 = Eigen::Matrix<double, 4, 4>::Identity();
    T_bbox_wrist3(1, 3)                       = 0.2622;

    wrist3.bounding_box.T = T_bbox_wrist3;

    _segment_structs.push_back(wrist3);

    /* ------------------------------------------ */

    // Distance to mounting point (0.0922m) adding distance to gripper bumper including force torque sensor distance extention (0.14m)
    robot_misc::Segment tool;
    tool.joint.tx     = 0.0;
    tool.joint.ty     = 0.0922;  //+ 0.14;
    tool.joint.tz     = 0.0;
    tool.joint.Rx     = 0.0;
    tool.joint.Ry     = 0.0;
    tool.joint.Rz     = M_PI / 2.0;
    tool.joint.offset = 0;
    tool.joint.fixed  = true;

    _segment_structs.push_back(tool);

    /* ------------------------------------------ */
    _initialized = true;
}

}  // namespace robot_description
}  // namespace mhp_robot
