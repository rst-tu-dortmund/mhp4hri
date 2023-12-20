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
#include <mhp_robot/robot_obstacle/obstacle_pipeline/tf_obstacle_process.h>

namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

TFObstacleProcess::TFObstacleProcess(const std::string& name) : BaseProcess(name) {}

bool TFObstacleProcess::process(std::map<int, robot_misc::Obstacle>& static_obstacles, std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                                std::map<int, robot_misc::Human>& humans, std::map<int, robot_misc::UtilityObject>& utility_objects,
                                std::map<int, robot_misc::Plane>& planes, bool forced)
{
    if (!_initialized)
    {
        ROS_WARN("TFObstacleProcess: Cannot process in an uninitialized state.");
        return false;
    }

    bool success = true;

    if (_active || forced)
    {
        int id = -1;

        // Fill dynamic obstacle poses with matching tf data
        for (auto& obs : dynamic_obstacles)
        {
            id = obs.second.id;

            if (id < 0)
            {
                ROS_WARN_THROTTLE(20, "TFObstacleProcess: Found obstacle with invalid ID (-1). Skipping...");
                success = false;
                continue;
            }

            std::string frame_name = getObstacleFrameName(id);

            if (_tf_listener.frameExists(frame_name))
            {
                Eigen::Matrix4d T = robot_misc::Common::getTransformation("world", frame_name, _tf_listener);

                if (obs.second.bounding_box.type == robot_misc::BoundingBoxType::EBOX)
                {
                    Eigen::Quaterniond quat(T.block<3, 3>(0, 0));
                    Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);

                    // Only update position and roation around z
                    obs.second.state._poses[0].block<3, 1>(0, 3) = T.block<3, 1>(0, 3);
                    obs.second.state._poses[0].block<3, 3>(0, 0) = robot_misc::Common::rotz(euler(2)).block<3, 3>(0, 0);
                }
                else
                {
                    // Update full pose
                    obs.second.state._poses[0] = T;
                }
            }
        }

        // Fill human body part poses with matching tf data
        _mutex_human.lock();
        for (auto& human : humans)
        {
            if (humans.size() > 1)
            {
                ROS_ERROR("TFObstacleProcess: Currently only one human is supported.");
            }

            for (auto& bdy : human.second._body_parts)
            {
                std::string frame_name = getHumanFrameName(human.second._id, human.second._name, bdy.second.name);
                if (_tf_listener.frameExists(frame_name))
                {
                    Eigen::Matrix4d T = robot_misc::Common::getTransformation("world", frame_name, _tf_listener);

                    // Update full pose for body part
                    bdy.second.state._poses[0] = T;

                    if (bdy.second.name.compare("Hip") == 0)
                    {
                        // Set Hip pose also to Leg Body part
                        human.second._body_parts["Leg"].state._poses[0] = T;

                        human.second._foot_print(0) = T(0, 3);
                        human.second._foot_print(1) = T(1, 3);
                        human.second._foot_print(2) = atan2(T(1, 0), T(0, 0));  // it's theta (rotation aroud y-axis)
                    }
                }
            }
            human.second._poses_updated = true;
        }
        _mutex_human.unlock();

        // Fill utility object poses with matching tf data
        for (auto& util : utility_objects)
        {
            id = util.second.id;

            if (id < 0)
            {
                ROS_WARN_THROTTLE(20, "TFObstacleProcess: Found utility object with invalid ID (-1). Skipping...");
                success = false;
                continue;
            }

            std::string frame_name = getUtilityObjectFrameName(id);

            if (_tf_listener.frameExists(frame_name))
            {
                Eigen::Matrix4d T = robot_misc::Common::getTransformation("world", frame_name, _tf_listener);

                if (util.second.bounding_box.type == robot_misc::BoundingBoxType::EBOX)
                {
                    Eigen::Quaterniond quat(T.block<3, 3>(0, 0));
                    Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);

                    // Only update position and roation around z
                    util.second.state._poses[0].block<3, 1>(0, 3) = T.block<3, 1>(0, 3);
                    util.second.state._poses[0].block<3, 3>(0, 0) = robot_misc::Common::rotz(euler(2)).block<3, 3>(0, 0);
                }
                else
                {
                    // Update full pose
                    util.second.state._poses[0] = T;
                }
            }
        }
    }

    return success;
}

std::string TFObstacleProcess::getObstacleFrameName(int id) const { return _obstacle_id_prefix_str + "_" + std::to_string(id); }

std::string TFObstacleProcess::getUtilityObjectFrameName(int id) const { return _object_id_prefix_str + "_" + std::to_string(id); }

std::string TFObstacleProcess::getHumanFrameName(int id, const std::string& human, const std::string& body) const
{
    //    return human + "_" + body + "_" + std::to_string(id);
    return human + body + "_" + std::to_string(id);
}

void TFObstacleProcess::setDefaultTransform(Eigen::Ref<Eigen::Matrix<double, 4, 4>> pose)
{
    pose = Eigen::Matrix<double, 4, 4>::Identity();
    pose.block<3, 1>(0, 3) << 0, -3, 1;
}

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot
