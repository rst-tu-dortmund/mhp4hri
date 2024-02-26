/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2024,
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
*  Authors: Maximilian Krämer, Heiko Renz
*  Maintainer(s)/Modifier(s): 
 *********************************************************************/
#ifndef ROBOT_INFORMATION_GAIN_SIMPLE_H
#define ROBOT_INFORMATION_GAIN_SIMPLE_H

#include <mhp_robot/robot_trajectory_optimization/robot_information_gain.h>

namespace mhp_robot {
namespace robot_trajectory_optimization {

class RobotInformationGainSimple : virtual public RobotInformationGain
{
 public:
    RobotInformationGainSimple() = default;

    using Ptr  = std::shared_ptr<RobotInformationGainSimple>;
    using UPtr = std::unique_ptr<RobotInformationGainSimple>;

    double computeGain(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k) override;
    void inverseDistanceWeigthing( const Eigen::Ref<const Eigen::Vector3d>& point, double& gain);

private:
    bool _first_pcl = true;
    Eigen::VectorXd _weighted_dists;
};

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot

#endif  // ROBOT_INFORMATION_GAIN_SIMPLE_H
