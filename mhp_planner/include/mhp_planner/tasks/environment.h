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
#ifndef SRC_TASKS_INCLUDE_MHP_PLANNER_TASKS_ENVIRONMENT_H_
#define SRC_TASKS_INCLUDE_MHP_PLANNER_TASKS_ENVIRONMENT_H_

#include <mhp_planner/controllers/controller_interface.h>
#include <mhp_planner/plants/plant_interface.h>
#include <memory>
#include <string>

namespace mhp_planner {

/**
 * @brief Standard environment for control tasks
 *
 * @ingroup tasks
 *
 * Usually, tasks are called with an Environment to facilitate the
 * initialization and verification of commonly used control architectures.
 * An Environment contains a plant, observer and controller.
 *
 * @see TaskInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class Environment
{
 public:
    using Ptr = std::shared_ptr<Environment>;

    //! Default constructor
    Environment() {}
    //! Construct environment with a controller and plant
    Environment(ControllerInterface::Ptr controller, PlantInterface::Ptr plant);

    //! Check if a controller has been specified
    bool hasController() const { return (bool)_controller; }
    //! Check if a plant has been specified
    bool hasPlant() const { return (bool)_plant; }

    //! Read access to the underlying controller
    const ControllerInterface::Ptr& getController() const { return _controller; }
    //! Write access to the underlying controller
    ControllerInterface::Ptr getControllerPtr() { return _controller; }
    //! Read access to the underlying plant
    const PlantInterface::Ptr& getPlant() const { return _plant; }
    //! Write access to the underlying plant
    PlantInterface::Ptr getPlantPtr() { return _plant; }

    //! Set controller
    void setController(ControllerInterface::Ptr controller);
    //! Set plant
    void setPlant(PlantInterface::Ptr plant);

    bool verify(std::string* msg = nullptr) const;

    //! Reset environment
    void reset();

 private:
    ControllerInterface::Ptr _controller;
    PlantInterface::Ptr _plant;
};

}  // namespace mhp_planner
#endif  // SRC_TASKS_INCLUDE_MHP_PLANNER_TASKS_ENVIRONMENT_H_