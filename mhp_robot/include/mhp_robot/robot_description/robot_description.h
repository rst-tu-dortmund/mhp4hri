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

#ifndef ROBOT_DESCRIPTION_H
#define ROBOT_DESCRIPTION_H

#include <mhp_robot/robot_misc/plane.h>
#include <mhp_robot/robot_misc/segment.h>
#include <urdf/model.h>
#include <Eigen/Dense>
#include <vector>

namespace mhp_robot {
namespace robot_description {

class RobotDescription
{
 public:
    using Ptr  = std::shared_ptr<RobotDescription>;
    using UPtr = std::unique_ptr<RobotDescription>;

    RobotDescription();
    RobotDescription(const std::string& urdf_param, const std::string& first_frame, const std::string& last_frame);

    RobotDescription(const RobotDescription&)            = delete;
    RobotDescription(RobotDescription&&)                 = default;
    RobotDescription& operator=(const RobotDescription&) = delete;
    RobotDescription& operator=(RobotDescription&&)      = default;
    virtual ~RobotDescription() {}

    virtual UPtr createUniqueInstance() const;
    virtual Ptr createSharedInstance() const;

    const std::vector<robot_misc::Segment>& getKinematicStructure() const;

    const urdf::Model& getURDFModel() const;
    bool isInitialized() const;

    const std::string& getURDFParam() const;
    const std::string& getFirstFrame() const;
    const std::string& getLastFrame() const;

    const robot_misc::Plane& getGroundPlane() const;
    const robot_misc::Plane& getRoofPlane() const;

    virtual bool parseURDF(const std::string& param, const std::string& first_frame, const std::string& last_frame);

 protected:
    virtual bool checkParameters() const;
    virtual void setDefaultPlanes(robot_misc::Plane& ground_plane, robot_misc::Plane& roof_plane);

    std::vector<robot_misc::Segment> _segment_structs;
    urdf::Model _urdf_model;

    robot_misc::Plane _ground_plane;
    robot_misc::Plane _roof_plane;

    bool _initialized = false;

 private:
    std::string _urdf_param  = "";
    std::string _first_frame = "";
    std::string _last_frame  = "";

    urdf::Link _first_link;

    robot_misc::Segment createSegment(const urdf::Link& link) const;
    robot_misc::BoundingBox createBoundingBox(const urdf::Link& link) const;
    robot_misc::Joint createJoint(const urdf::Joint& joint) const;

    bool hasFrame(const std::string& frame, const urdf::Link& target_link, urdf::Link& link, bool construct = false);

    bool extractGroundPlane(const urdf::Link& world_link, robot_misc::Plane& ground_plane) const;
    bool extractRoofPlane(const urdf::Link& world_link, robot_misc::Plane& roof_plane) const;
};

}  // namespace robot_description
}  // namespace mhp_robot

#endif  // ROBOT_DESCRIPTION_H
