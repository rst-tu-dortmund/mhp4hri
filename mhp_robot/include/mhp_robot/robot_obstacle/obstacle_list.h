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

#ifndef OBSTACLE_LIST_H
#define OBSTACLE_LIST_H

#include <mhp_robot/robot_misc/common.h>
#include <mhp_robot/robot_misc/human.h>
#include <mhp_robot/robot_misc/obstacle.h>
#include <mhp_robot/robot_misc/plane.h>
#include <mhp_robot/robot_misc/utility_object.h>
#include <ros/ros.h>
#include <mutex>
#include <vector>

namespace mhp_robot {
namespace robot_obstacle {

class ObstacleList
{
 public:
    ObstacleList() = default;

    ObstacleList(const ObstacleList&)            = delete;
    ObstacleList(ObstacleList&&)                 = delete;
    ObstacleList& operator=(const ObstacleList&) = delete;
    ObstacleList& operator=(ObstacleList&&)      = delete;
    ~ObstacleList()                              = default;

    int staticObstacleCount() const;
    int dynamicObstacleCount() const;
    int temporaryStaticObstacleCount() const;
    int humanCount() const;
    int planeCount() const;
    int utilityObjectCount() const;

    static std::vector<robot_misc::Obstacle> _static_obstacles;
    static std::vector<robot_misc::Obstacle> _dynamic_obstacles;
    static std::vector<robot_misc::Human> _humans;
    static std::vector<robot_misc::UtilityObject> _utility_objects;
    static std::vector<robot_misc::Plane> _planes;

    static void createObstacleList(const XmlRpc::XmlRpcValue& param, std::vector<robot_misc::Obstacle>& list);
    static std::vector<robot_misc::Obstacle> createObstacleList(const XmlRpc::XmlRpcValue& param);

    static void createObstacleMap(const XmlRpc::XmlRpcValue& param, std::map<int, robot_misc::Obstacle>& map);
    static std::map<int, robot_misc::Obstacle> createObstacleMap(const XmlRpc::XmlRpcValue& param);

    static void createHumanList(const XmlRpc::XmlRpcValue& param, std::vector<robot_misc::Human>& list);
    static std::vector<robot_misc::Human> createHumanList(const XmlRpc::XmlRpcValue& param);

    static void createHumanMap(const XmlRpc::XmlRpcValue& param, std::map<int, robot_misc::Human>& map);
    static std::map<int, robot_misc::Human> createHumanMap(const XmlRpc::XmlRpcValue& param);

    static void createUtilityObjectList(const XmlRpc::XmlRpcValue& param, std::vector<robot_misc::UtilityObject>& list);
    static std::vector<robot_misc::UtilityObject> createUtilityObjectList(const XmlRpc::XmlRpcValue& param);

    static void createUtilityObjectMap(const XmlRpc::XmlRpcValue& param, std::map<int, robot_misc::UtilityObject>& map);
    static std::map<int, robot_misc::UtilityObject> createUtilityObjectMap(const XmlRpc::XmlRpcValue& param);

    static void createPlaneList(const XmlRpc::XmlRpcValue& param, std::vector<robot_misc::Plane>& list);
    static std::vector<robot_misc::Plane> createPlaneList(const XmlRpc::XmlRpcValue& param);

    static void createPlaneMap(const XmlRpc::XmlRpcValue& param, std::map<int, robot_misc::Plane>& map);
    static std::map<int, robot_misc::Plane> createPlaneMap(const XmlRpc::XmlRpcValue& param);

    static std::mutex _mutex;

 private:
    using BoundingBoxType = robot_misc::BoundingBoxType;

    static std::vector<std::string> _human_body_names;

    static std::string forceDecimal(double in, int precision);
};

}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // OBSTACLE_LIST_H
