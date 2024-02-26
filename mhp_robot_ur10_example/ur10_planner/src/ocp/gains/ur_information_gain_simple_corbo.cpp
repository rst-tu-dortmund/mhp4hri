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

#include <ur10_planner/ocp/gains/ur_information_gain_simple_corbo.h>

namespace mhp_planner {
mhp_planner::URBaseInformationGain::Ptr URInformationGainSimple::getInstance() const { return std::make_shared<URInformationGainSimple>(); }
bool URInformationGainSimple::fromParameterServer(const std::string& ns)
{
    ros::NodeHandle nh;

    // Set weight for the information gain
    nh.getParam(ns + "/weight", _w_gain);
    
    return true;
}
}  // namespace corbo