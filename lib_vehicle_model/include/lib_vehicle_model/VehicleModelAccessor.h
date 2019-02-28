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

#include <stdexcept>
#include <vector>
#include <string>
#include <dlfcn.h>
#include <memory>
#include <stdlib.h>
#include <sstream>
#include <mutex>
#include "VehicleState.h"
#include "VehicleMotionModel.h"
#include "VehicleModelControlInput.h"
#include "ParameterServer.h"

namespace lib_vehicle_model {
  /**
   * @namespace VehicleModelAccessor
   * @brief Namespace which controls the interface to the dynamically loaded vehicle models used to predict host vehicle motion. 
   * 
   * A link to the parameter server is provided in the init() function and this is used to load the appropriate vehicle model. 
   * When a plugin or guidance component calls the predict functions of the VehicleModelAccessor does basic input validation checks and then passes the request onto the loaded vehicle model. 
   * 
   * NOTE: The init() function must be called once before using the vehicle model
   *       If the init() function is called more than once an exception will be thrown and the vehicle model will NOT be re-loaded
   */
  namespace VehicleModelAccessor
  {
    //
    // Private Namespace
    //
    namespace {
      // Typedef for function pointers to use with loaded libraries create and destroy functions
      typedef VehicleMotionModel* (*create_fnc_ptr)();
      typedef void (*destroy_fnc_ptr)(VehicleMotionModel*);

      /** 
       * @brief Helper function to load the host vehicle model. Must be called only in constructor
       * 
       * @throws std::invalid_argument If the model could not be loaded 
       */
      void loadModel();

      /**
       * @brief Helper function to validate the initial vehicle state for a motion prediction
       * 
       * @param initial_state The starting state of the vehicle passed into the prediction function
       * 
       * @throws std::invalid_argument If the initial vehicle state is found to be invalid
       */
      void validateInitialState(const VehicleState& initial_state);  

      /**
       * @brief Helper function to validate the control inputs for a motion prediction
       * 
       * @param initial_state The starting state of the vehicle passed into the prediction function
       * @param control_inputs The control inputs for the vehicle passed into the prediction function
       * @param timestep The difference in time between successive control inputs in seconds
       * 
       * @throws std::invalid_argument If the initial control inputs are found to be invalid
       */
      void validateControlInputs(const VehicleState& initial_state, const std::vector<VehicleModelControlInput>& control_inputs, double timestep);  
    }


    //
    // Public Namespace
    //

    /**
     * @brief Initialization function for class. Loads the library as specified by ros parameters 
     * 
     * @param parameter_server A reference to the parameter server which vehicle models will use to load parameters
     * 
     * @throws std::invalid_argument If the model could not be loaded or parameters could not be read
     * @throws std::runtime_error If this function is called more than once within the same process execution
     * 
     */ 
    void init(std::shared_ptr<ParameterServer> parameter_server);

    //
    // Functions matching the VehicleMotionModel interface
    //

    /**
     * @brief Predict vehicle motion assuming no change in control input
     * 
     * @param initial_state The starting state of the vehicle
     * @param timestep The time increment between returned traversed states
     * @param delta_t The time to project the motion forward for
     * 
     * @return A list of traversed states seperated by the timestep
     * 
     * @throws std::runtime_error If this function is called before the init() function
     * 
     * NOTE: This function header must match a predict function found in the VehicleMotionModel interface
     * 
     */
    std::vector<VehicleState> predict(VehicleState initial_state,
      double timestep, double delta_t); 

    /**
     * @brief Predict vehicle motion given a starting state and list of control inputs
     * 
     * @param initial_state The starting state of the vehicle
     * @param control_inputs A list of control inputs seperated by the provided timestep
     * @param timestep The time increment between returned traversed states and provided control inputs
     * 
     * @return A list of traversed states seperated by the timestep
     * 
     * @throws std::runtime_error If this function is called before the init() function
     * 
     * NOTE: This function header must match a predict function found in the VehicleMotionModel interface
     * 
     */
    std::vector<VehicleState> predict(VehicleState initial_state,
      std::vector<VehicleModelControlInput> control_inputs, double timestep);
  };
}