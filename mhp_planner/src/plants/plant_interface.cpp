/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020,
 *  TU Dortmund University, Institute of Control Theory and System Enginnering
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
 *  Authors: Christoph Rösmann
 *  Modifier(s)/Maintainer(s): Maximilian Krämer, Heiko Renz
 *********************************************************************/

#include <mhp_planner/plants/plant_interface.h>

namespace mhp_planner {

bool PlantInterface::control(const PlantInterface::ControlVector& u, const Duration& dt, const Time& t, const std::string& ns)
{
    TimeSeries::Ptr u_sequence = std::make_shared<TimeSeries>();
    TimeSeries::Ptr x_sequence = std::make_shared<TimeSeries>();

    u_sequence->add(0.0, u);

    return control(u_sequence, x_sequence, dt, t, ns);
}

}  // namespace mhp_planner
