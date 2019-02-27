#pragma once
/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <vector>
#include "VehicleState.h"
#include "VehicleModelControlInput.h"

namespace lib_vehicle_model {
  /**
   * @class VehicleMotionPredictor
   * @brief An interface which defines the predict functions used for predicting host vehicle motion.
   * 
   * Interface is used to ensure consistency between the VehicleModelAccessor interfaces and the implemented vehicle models.
   */
  class VehicleMotionPredictor 
  {
    public:

      /**
       * @brief Pure virtual destructor to ensure delete safety for pointers to implementing classes
       * 
       */
      virtual ~VehicleMotionPredictor() = 0;

      /**
       * @brief Predict vehicle motion assuming no change in control input
       * 
       * @param initial_state The starting state of the vehicle
       * @param timestep The time increment between returned traversed states
       * @param delta_t The time to project the motion forward for
       * 
       * @return A list of traversed states seperated by the timestep
       * 
       */
      virtual std::vector<VehicleState> predict(VehicleState initial_state,
        double timestep, double delta_t) = 0; // Defined as pure virtual function

      /**
       * @brief Predict vehicle motion given a starting state and list of control inputs
       * 
       * @param initial_state The starting state of the vehicle
       * @param control_inputs A list of control inputs seperated by the provided timestep
       * @param timestep The time increment between returned traversed states and provided control inputs
       * 
       * @return A list of traversed states seperated by the timestep
       * 
       */
      virtual std::vector<VehicleState> predict(VehicleState initial_state,
        std::vector<VehicleModelControlInput> control_inputs, double timestep) = 0; // Defined as pure virtual function
  };
}