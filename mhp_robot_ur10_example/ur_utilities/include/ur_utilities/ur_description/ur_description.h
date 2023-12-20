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

#ifndef UR_DESCRIPTION_H
#define UR_DESCRIPTION_H

#include <mhp_robot/robot_description/robot_description.h>

namespace mhp_robot {
namespace robot_description {

class URDescription : public RobotDescription
{
 public:
    using Ptr  = std::shared_ptr<URDescription>;
    using UPtr = std::unique_ptr<URDescription>;

    URDescription() = default;
    URDescription(const std::string& urdf_param, const std::string& first_frame, const std::string& last_frame);

    RobotDescription::UPtr createUniqueInstance() const override;
    RobotDescription::Ptr createSharedInstance() const override;

 private:
    bool checkParameters() const override;

    // Segments are assumed to be in this order for ur_ classes
    std::vector<std::string> _link_names{
        "pedestal_link", "base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "ee_link",
    };
};

}  // namespace robot_description
}  // namespace mhp_robot

#endif  // UR_DESCRIPTION_H
