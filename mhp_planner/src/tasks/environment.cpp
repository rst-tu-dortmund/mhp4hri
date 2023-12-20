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

#include <mhp_planner/tasks/environment.h>

#include <string>

namespace mhp_planner {

Environment::Environment(ControllerInterface::Ptr controller, PlantInterface::Ptr plant)
{
    setController(controller);
    setPlant(plant);
}

void Environment::setController(ControllerInterface::Ptr controller) { _controller = controller; }

void Environment::setPlant(PlantInterface::Ptr plant) { _plant = plant; }

bool Environment::verify(std::string* msg) const
{
    if (msg) msg->clear();
    bool ret_val = true;
    // first check pointer
    if (!hasController())
    {
        ret_val = false;
        if (msg) *msg += "Controller not specified.\n";
    }

    if (!hasPlant())
    {
        ret_val = false;
        if (msg) *msg += "Plant not specified.\n";
    }
    if (!ret_val) return false;

    // now check dimensions
    int controller_state_dim   = getController()->getStateDimension();
    int controller_control_dim = getController()->getControlInputDimension();

    int plant_input_dim  = getPlant()->getInputDimension();
    int plant_output_dim = getPlant()->getOutputDimension();

    // control input vs. plant input
    if (controller_control_dim != plant_input_dim)
    {
        ret_val = false;
        if (msg)
        {
            *msg += "Contol input dimension (" + std::to_string(controller_control_dim) + ") does not match plant intput dimension (" +
                    std::to_string(plant_input_dim) + ").\n";
        }
    }
    return ret_val;
}

void Environment::reset()
{
    if (_controller) _controller->reset();
    if (_plant) _plant->reset();
}
}  // namespace mhp_planner
