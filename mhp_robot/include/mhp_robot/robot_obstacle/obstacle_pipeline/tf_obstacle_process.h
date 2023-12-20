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

#ifndef TF_OBSTACLE_PROCESS_H
#define TF_OBSTACLE_PROCESS_H

#include <mhp_robot/robot_obstacle/obstacle_pipeline/base_process.h>
#include <tf/transform_listener.h>
#include <mutex>

namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

class TFObstacleProcess : public BaseProcess
{
 public:
    using Ptr  = std::shared_ptr<TFObstacleProcess>;
    using UPtr = std::unique_ptr<TFObstacleProcess>;

    TFObstacleProcess(const std::string& name);

    bool process(std::map<int, robot_misc::Obstacle>& static_obstacles, std::map<int, robot_misc::Obstacle>& dynamic_obstacles,
                 std::map<int, robot_misc::Human>& humans, std::map<int, robot_misc::UtilityObject>& utility_objects,
                 std::map<int, robot_misc::Plane>& planes, bool forced = false) override;

 private:
    tf::TransformListener _tf_listener;

    std::string _obstacle_id_prefix_str = "obs";
    std::string _object_id_prefix_str   = "util";

    std::string getObstacleFrameName(int id) const;
    std::string getUtilityObjectFrameName(int id) const;
    std::string getHumanFrameName(int id, const std::string& human, const std::string& body) const;
    void setDefaultTransform(Eigen::Ref<Eigen::Matrix<double, 4, 4>> pose);

    std::mutex _mutex_human;
};

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot

#endif  // TF_OBSTACLE_PROCESS_H
