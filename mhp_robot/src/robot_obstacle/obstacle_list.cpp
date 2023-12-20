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
#include <mhp_robot/robot_obstacle/obstacle_list.h>

namespace mhp_robot {
namespace robot_obstacle {

std::mutex ObstacleList::_mutex;
std::vector<robot_misc::Obstacle> ObstacleList::_static_obstacles;
std::vector<robot_misc::Obstacle> ObstacleList::_dynamic_obstacles;
std::vector<robot_misc::Human> ObstacleList::_humans;
std::vector<robot_misc::UtilityObject> ObstacleList::_utility_objects;
std::vector<robot_misc::Plane> ObstacleList::_planes;
std::vector<std::string> ObstacleList::_human_body_names = {"Head", "Neck", "Hip", "RUArm", "LUArm", "RFArm", "LFArm", "RHand", "LHand"};

int ObstacleList::staticObstacleCount() const { return _static_obstacles.size(); }

int ObstacleList::dynamicObstacleCount() const { return _dynamic_obstacles.size(); }

int ObstacleList::temporaryStaticObstacleCount() const
{
    return std::count_if(_dynamic_obstacles.begin(), _dynamic_obstacles.end(), [](robot_misc::Obstacle& o) { return o.temporary_static; });
}

int ObstacleList::humanCount() const { return _humans.size(); }

int ObstacleList::planeCount() const { return _planes.size(); }

int ObstacleList::utilityObjectCount() const { return _utility_objects.size(); }

void ObstacleList::createObstacleList(const XmlRpc::XmlRpcValue& param, std::vector<robot_misc::Obstacle>& list)
{
    list.clear();

    if (param.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        // Iterate over obstacles
        for (int i = 0; i < param.size(); ++i)
        {
            const XmlRpc::XmlRpcValue& obs_el = param[i];

            if (obs_el.getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                robot_misc::Obstacle obstacle;
                bool radius = false, translation = false, rotation = false, id = false, name = false;
                Eigen::Matrix3d Rx, Ry, Rz;

                // Iterate over properties
                for (XmlRpc::XmlRpcValue::const_iterator prop_it = obs_el.begin(); prop_it != obs_el.end(); ++prop_it)
                {
                    if (prop_it->first == "radius" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    {
                        obstacle.bounding_box.radius = prop_it->second;
                        radius                       = true;
                    }
                    if (prop_it->first == "length_x" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    {
                        obstacle.bounding_box.length_x = std::max((double)prop_it->second, 0.0);
                    }
                    if (prop_it->first == "length_y" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    {
                        obstacle.bounding_box.length_y = std::max((double)prop_it->second, 0.0);
                    }
                    if (prop_it->first == "extruded" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                    {
                        obstacle.bounding_box.extruded = prop_it->second;
                    }
                    if (prop_it->first == "translation" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray && prop_it->second.size() == 3)
                    {
                        obstacle.state._poses[0].block<4, 1>(0, 3) << prop_it->second[0], prop_it->second[1], prop_it->second[2], 1.0;
                        translation = true;
                    }
                    if (prop_it->first == "rotation" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray && prop_it->second.size() == 3)
                    {
                        Rz       = robot_misc::Common::rotz(prop_it->second[0]).block<3, 3>(0, 0);
                        Ry       = robot_misc::Common::roty(prop_it->second[1]).block<3, 3>(0, 0);
                        Rx       = robot_misc::Common::rotx(prop_it->second[2]).block<3, 3>(0, 0);
                        rotation = true;
                    }
                    if (prop_it->first == "id" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
                    {
                        obstacle.id = prop_it->second;
                        id          = true;
                    }
                    if (prop_it->first == "name" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeString)
                    {
                        obstacle.name = static_cast<std::string>(prop_it->second);
                        name          = true;
                    }
                    if (prop_it->first == "group_id" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
                    {
                        obstacle.group_id = prop_it->second;
                    }
                    if (prop_it->first == "gazebo" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                    {
                        obstacle.gazebo = prop_it->second;
                    }
                }

                if (translation && rotation && id && name)
                {
                    // Check whether the same ID already on the list
                    auto obs_iter = std::find_if(list.begin(), list.end(), [&](const robot_misc::Obstacle& obs) { return obstacle.id == obs.id; });

                    if (obs_iter == list.end())
                    {
                        if (obstacle.bounding_box.length_x > 0.0)
                        {
                            if (obstacle.bounding_box.length_y > 0.0)
                            {
                                obstacle.bounding_box.type = (obstacle.bounding_box.extruded ? BoundingBoxType::EBOX : BoundingBoxType::BOX);
                            }
                            else
                            {
                                obstacle.bounding_box.type = BoundingBoxType::CYLINDER;
                            }
                        }
                        else
                        {
                            obstacle.bounding_box.type = BoundingBoxType::SPHERE;
                        }

                        if (obstacle.bounding_box.type == BoundingBoxType::EBOX)
                        {
                            obstacle.state._poses[0].block<3, 3>(0, 0) = Rz;
                            list.push_back(obstacle);
                        }
                        else
                        {
                            if (!radius)
                            {
                                ROS_WARN("ObstacleList: Not all obstacle properties were set. obstacle skipped.");
                            }
                            else
                            {
                                obstacle.state._poses[0].block<3, 3>(0, 0).noalias() = Rz * Ry * Rx;
                                list.push_back(obstacle);
                            }
                        }
                    }
                    else
                    {
                        ROS_WARN("ObstacleList: Obstacle with ID %d already exists. Obstacle skipped.", obstacle.id);
                    }
                }
                else
                {
                    ROS_WARN("ObstacleList: Not all obstacle properties were set. Obstacle skipped.");
                }
            }
            else
            {
                ROS_WARN("ObstacleList: Invalid obstacle format. Obstacle skipped.");
            }
        }
    }
    else
    {
        ROS_WARN("ObstacleList: No obstacles found.");
    }
}

std::vector<robot_misc::Obstacle> ObstacleList::createObstacleList(const XmlRpc::XmlRpcValue& param)
{
    std::vector<robot_misc::Obstacle> obstacle_list;
    createObstacleList(param, obstacle_list);
    return obstacle_list;
}

void ObstacleList::createObstacleMap(const XmlRpc::XmlRpcValue& param, std::map<int, robot_misc::Obstacle>& map)
{
    map.clear();
    std::vector<robot_misc::Obstacle> obstacle_list = createObstacleList(param);

    for (auto const& obs : obstacle_list)
    {
        map.insert(std::pair<int, robot_misc::Obstacle>(obs.id, obs));
    }
}

std::map<int, robot_misc::Obstacle> ObstacleList::createObstacleMap(const XmlRpc::XmlRpcValue& param)
{
    std::map<int, robot_misc::Obstacle> obstacle_map;
    createObstacleMap(param, obstacle_map);
    return obstacle_map;
}

void ObstacleList::createHumanList(const XmlRpc::XmlRpcValue& param, std::vector<robot_misc::Human>& list)
{
    list.clear();
    if (param.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        // Iterate over humans
        for (int i = 0; i < param.size(); ++i)
        {
            XmlRpc::XmlRpcValue hum;
            hum            = param[i];
            int id_for_hum = 0;
            std::string name_for_hum;
            std::vector<double> radius;
            std::vector<double> length;
            std::vector<double> angles;
            // Iterate over Elements in each Human
            for (XmlRpc::XmlRpcValue::const_iterator elem_it = hum.begin(); elem_it != hum.end(); ++elem_it)
            {
                if (elem_it->first == "id" && elem_it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    id_for_hum = elem_it->second;
                }
                if (elem_it->first == "name" && elem_it->second.getType() == XmlRpc::XmlRpcValue::TypeString)
                {
                    name_for_hum = static_cast<std ::string>(elem_it->second);
                }
                if (elem_it->first == "length" && elem_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray)
                {
                    if (elem_it->second.size() == 10)
                    {
                        length = {elem_it->second[0], elem_it->second[1], elem_it->second[2], elem_it->second[3], elem_it->second[4],
                                  elem_it->second[5], elem_it->second[6], elem_it->second[7], elem_it->second[8], elem_it->second[9]};
                    }
                    else
                    {
                        ROS_WARN("HumanList: Number of lengths not suitable. Use default Length");
                        length = {0.95, 0.55, 0.0, 0.3, 0.36, 0.0, 0.3, 0.36, 0.0, 0.95};
                    }
                }
                if (elem_it->first == "radius" && elem_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray)
                {
                    if (elem_it->second.size() == 10)
                    {
                        radius = {elem_it->second[0], elem_it->second[1], elem_it->second[2], elem_it->second[3], elem_it->second[4],
                                  elem_it->second[5], elem_it->second[6], elem_it->second[7], elem_it->second[8], elem_it->second[9]};
                    }
                    else
                    {
                        ROS_WARN("HumanList: Number of radius not suitable. Use default radius");
                        radius = {0.2, 0.2, 0.12, 0.06, 0.06, 0.0, 0.06, 0.06, 0.0, 0.2};
                    }
                }
                if (elem_it->first == "joint_angles" && elem_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray)
                {
                    if (elem_it->second.size() == 11)
                    {
                        angles = {elem_it->second[0], elem_it->second[1], elem_it->second[2], elem_it->second[3],
                                  elem_it->second[4], elem_it->second[5], elem_it->second[6], elem_it->second[7],
                                  elem_it->second[8], elem_it->second[9], elem_it->second[10]};
                    }
                    else
                    {
                        ROS_WARN("HumanList: Number of joint_angles not suitable. Use default angles");
                        angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    }
                }
            }

            robot_misc::Human human(id_for_hum, name_for_hum, length, radius, angles);

            bool human_id_set;
            if (human._id == id_for_hum)
            {
                human_id_set = true;
            }
            else
            {
                human_id_set = false;
            }

            if (human_id_set)
            {
                // Check whether the same ID already on the list
                auto hum_iter = std::find_if(list.begin(), list.end(), [&](const robot_misc::Human& hum) { return human._id == hum._id; });

                if (hum_iter == list.end())
                {
                    // Group body parts
                    for (auto& bdy : human._body_parts)
                    {
                        bdy.second.group_id = human._id;
                    }

                    // Finally add human to list
                    list.push_back(human);
                }

                else
                {
                    ROS_WARN("HumanConfig: Human with ID %d already exists. Human skipped.", human._id);
                }
            }
        human_end:;
        }
    }
    else
    {
        ROS_WARN("ObstacleList: No humans found.");
    }
}

std::vector<robot_misc::Human> ObstacleList::createHumanList(const XmlRpc::XmlRpcValue& param)
{
    std::vector<robot_misc::Human> list;
    createHumanList(param, list);
    return list;
}

void ObstacleList::createHumanMap(const XmlRpc::XmlRpcValue& param, std::map<int, robot_misc::Human>& map)
{
    map.clear();
    std::vector<robot_misc::Human> human_list = createHumanList(param);

    for (auto const& hum : human_list)
    {
        map.insert(std::pair<int, robot_misc::Human>(hum._id, hum));
    }
}

std::map<int, robot_misc::Human> ObstacleList::createHumanMap(const XmlRpc::XmlRpcValue& param)
{
    std::map<int, robot_misc::Human> map;
    createHumanMap(param, map);
    return map;
}

void ObstacleList::createUtilityObjectList(const XmlRpc::XmlRpcValue& param, std::vector<robot_misc::UtilityObject>& list)
{
    list.clear();

    if (param.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        // Iterate over UtilityObjects
        for (int i = 0; i < param.size(); ++i)
        {
            const XmlRpc::XmlRpcValue& obs_el = param[i];

            if (obs_el.getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                robot_misc::UtilityObject object;
                bool radius = false, translation = false, rotation = false, id = false, name = false;
                Eigen::Matrix3d Rx, Ry, Rz;

                // Iterate over properties
                for (XmlRpc::XmlRpcValue::const_iterator prop_it = obs_el.begin(); prop_it != obs_el.end(); ++prop_it)
                {
                    if (prop_it->first == "radius" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    {
                        object.bounding_box.radius = prop_it->second;
                        radius                     = true;
                    }
                    if (prop_it->first == "length_x" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    {
                        object.bounding_box.length_x = std::max((double)prop_it->second, 0.0);
                    }
                    if (prop_it->first == "length_y" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    {
                        object.bounding_box.length_x = std::max((double)prop_it->second, 0.0);
                    }
                    if (prop_it->first == "extruded" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                    {
                        object.bounding_box.extruded = prop_it->second;
                    }
                    if (prop_it->first == "translation" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray && prop_it->second.size() == 3)
                    {
                        object.state._poses[0].block<4, 1>(0, 3) << prop_it->second[0], prop_it->second[1], prop_it->second[2], 1.0;
                        translation = true;
                    }
                    if (prop_it->first == "rotation" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray && prop_it->second.size() == 3)
                    {
                        Rz       = robot_misc::Common::rotz(prop_it->second[0]).block<3, 3>(0, 0);
                        Ry       = robot_misc::Common::roty(prop_it->second[1]).block<3, 3>(0, 0);
                        Rx       = robot_misc::Common::rotx(prop_it->second[2]).block<3, 3>(0, 0);
                        rotation = true;
                    }
                    if (prop_it->first == "id" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
                    {
                        object.id = prop_it->second;
                        id        = true;
                    }
                    if (prop_it->first == "name" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeString)
                    {
                        object.name = static_cast<std::string>(prop_it->second);
                        name        = true;
                    }
                    if (prop_it->first == "group_id" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
                    {
                        object.group_id = prop_it->second;
                    }
                }

                if (translation && rotation && id && name)
                {
                    // Check whether the same ID already on the list
                    auto obs_iter = std::find_if(list.begin(), list.end(), [&](const robot_misc::UtilityObject& obs) { return object.id == obs.id; });

                    if (obs_iter == list.end())
                    {
                        if (object.bounding_box.length_x > 0.0)
                        {
                            if (object.bounding_box.length_y > 0.0)
                            {
                                object.bounding_box.type = (object.bounding_box.extruded ? BoundingBoxType::EBOX : BoundingBoxType::BOX);
                            }
                            else
                            {
                                object.bounding_box.type = BoundingBoxType::CYLINDER;
                            }
                        }
                        else
                        {
                            object.bounding_box.type = BoundingBoxType::SPHERE;
                        }

                        if (object.bounding_box.type == BoundingBoxType::EBOX)
                        {
                            object.state._poses[0].block<3, 3>(0, 0) = Rz;
                            list.push_back(object);
                        }
                        else
                        {
                            if (!radius)
                            {
                                ROS_WARN("ObstacleList: Not all object properties were set. Object skipped.");
                            }
                            else
                            {
                                object.state._poses[0].block<3, 3>(0, 0).noalias() = Rz * Ry * Rx;
                                list.push_back(object);
                            }
                        }
                    }
                    else
                    {
                        ROS_WARN("ObstacleList: Object with ID %d already exists. Object skipped.", object.id);
                    }
                }
                else
                {
                    ROS_WARN("ObstacleList: Not all object properties were set. Object skipped.");
                }
            }
            else
            {
                ROS_WARN("ObstacleList: Invalid object format. Object skipped.");
            }
        }
    }
    else
    {
        ROS_WARN("ObstacleList: No objects found.");
    }
}

std::vector<robot_misc::UtilityObject> ObstacleList::createUtilityObjectList(const XmlRpc::XmlRpcValue& param)
{
    std::vector<robot_misc::UtilityObject> object_list;
    createUtilityObjectList(param, object_list);
    return object_list;
}

void ObstacleList::createUtilityObjectMap(const XmlRpc::XmlRpcValue& param, std::map<int, robot_misc::UtilityObject>& map)
{
    map.clear();
    std::vector<robot_misc::UtilityObject> object_list = createUtilityObjectList(param);

    for (auto const& obj : object_list)
    {
        map.insert(std::pair<int, robot_misc::UtilityObject>(obj.id, obj));
    }
}

std::map<int, robot_misc::UtilityObject> ObstacleList::createUtilityObjectMap(const XmlRpc::XmlRpcValue& param)
{
    std::map<int, robot_misc::UtilityObject> object_map;
    createUtilityObjectMap(param, object_map);
    return object_map;
}

void ObstacleList::createPlaneList(const XmlRpc::XmlRpcValue& param, std::vector<robot_misc::Plane>& list)
{
    list.clear();

    if (param.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        // Iterate over planes
        for (int i = 0; i < param.size(); ++i)
        {
            const XmlRpc::XmlRpcValue& plane_param = param[i];

            if (plane_param.getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                robot_misc::Plane plane;
                bool id = false, name = false, point = false, normal = false;

                // Iterate over properties
                for (XmlRpc::XmlRpcValue::const_iterator prop_it = plane_param.begin(); prop_it != plane_param.end(); ++prop_it)
                {
                    if (prop_it->first == "support_vector" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray &&
                        prop_it->second.size() == 3)
                    {
                        plane.q << prop_it->second[0], prop_it->second[1], prop_it->second[2];
                        point = true;
                    }
                    if (prop_it->first == "normal_vector" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray &&
                        prop_it->second.size() == 3)
                    {
                        plane.n << prop_it->second[0], prop_it->second[1], prop_it->second[2];
                        plane.n.normalize();
                        normal = true;
                    }
                    if (prop_it->first == "name" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeString)
                    {
                        plane.name = static_cast<std::string>(prop_it->second);
                        name       = true;
                    }
                    if (prop_it->first == "id" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
                    {
                        plane.id = prop_it->second;
                        id       = true;
                    }
                    if (prop_it->first == "group_id" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
                    {
                        plane.group_id = prop_it->second;
                    }
                    if (prop_it->first == "gazebo" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                    {
                        plane.gazebo = prop_it->second;
                    }
                }

                if (name && id && point && normal)
                {
                    // Check whether the same ID already on the list
                    auto obs_iter = std::find_if(list.begin(), list.end(), [&](const robot_misc::Plane& p) { return plane.id == p.id; });

                    if (obs_iter == list.end())
                    {
                        list.push_back(plane);
                    }
                    else
                    {
                        ROS_WARN("ObstacleList: Plane with ID %d already exists. Plane skipped.", plane.id);
                    }
                }
                else
                {
                    ROS_WARN("ObstacleList: Not all plane properties were set. Plane skipped.");
                }
            }
            else
            {
                ROS_WARN("ObstacleList: Invalid plane format. Plane skipped.");
            }
        }
    }
    else
    {
        ROS_WARN("ObstacleList: No planes found.");
    }
}

std::vector<robot_misc::Plane> ObstacleList::createPlaneList(const XmlRpc::XmlRpcValue& param)
{
    std::vector<robot_misc::Plane> plane_list;
    createPlaneList(param, plane_list);
    return plane_list;
}

void ObstacleList::createPlaneMap(const XmlRpc::XmlRpcValue& param, std::map<int, robot_misc::Plane>& map)
{
    map.clear();
    std::vector<robot_misc::Plane> plane_list = createPlaneList(param);

    for (auto const& p : plane_list)
    {
        map.insert(std::pair<int, robot_misc::Plane>(p.id, p));
    }
}

std::map<int, robot_misc::Plane> ObstacleList::createPlaneMap(const XmlRpc::XmlRpcValue& param)
{
    std::map<int, robot_misc::Plane> plane_map;
    createPlaneMap(param, plane_map);
    return plane_map;
}

std::string ObstacleList::forceDecimal(double in, int precision)
{
    double rounded = in * pow(10, precision);
    rounded        = std::round(rounded);
    rounded /= pow(10, precision);

    std::stringstream out;

    out << rounded;
    if (std::round(rounded) == rounded)
    {
        // Rounding to precision made value an integer, append decimal .0
        out << ".0";
    }
    return out.str();
}

}  // namespace robot_obstacle
}  // namespace mhp_robot
