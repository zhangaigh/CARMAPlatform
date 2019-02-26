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
#include "VehicleState.h"
#include "VehicleMotionPredictor.h"
#include "VehicleMotionModel.h"
#include "VehicleModelControlInput.h"
#include "ParameterServer.h"




/**
 * @class VehicleModelAccessor
 * @brief Class which controls the interface to the dynamically loaded vehicle models used to predict host vehicle motion. 
 * 
 * A link to the parameter server is provided in the constructor and this is used to load the appropriate vehicle model. 
 * When a plugin or guidance component calls the predict functions of the VehicleModelAccessor does basic input validation checks and then passes the request onto the loaded vehicle model. 
 */
class VehicleModelAccessor: public VehicleMotionPredictor
{
  private:
    std::shared_ptr<ParameterServer> param_server_;

    // Parameters
    std::string vehicle_model_lib_path_;
    double max_forward_speed_;
    double max_reverse_speed_;
    double forward_acceleration_limit_;
    double forward_deceleration_limit_;
    double reverse_acceleration_limit_;
    double reverse_deceleration_limit_;
    double max_steering_angle_;
    double min_steering_angle_;
    double max_steering_angle_rate_;
    double max_trailer_angle_;
    double min_trailer_angle_;

    // Vehicle Model
    std::shared_ptr<VehicleMotionModel> vehicle_model_;

    // Typedef for function pointers to use with loaded libraries create and destroy functions
    typedef VehicleMotionModel* (*create_fnc_ptr)();
    typedef void (*destroy_fnc_ptr)(VehicleMotionModel*);

    // The pointers to the loaded libraries create and destroy functions
    create_fnc_ptr create_fnc_;
    destroy_fnc_ptr destroy_fnc_;



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

  public:

    /**
     * @brief Constructor 
     * 
     * @param parameter_server A reference to the parameter server which vehicle models will use to load parameters
     * 
     * @throws std::invalid_argument If the model could not be loaded or parameters could not be read
     * 
     */ 
    VehicleModelAccessor(std::shared_ptr<ParameterServer> parameter_server);

    /**
     * @brief Destructor as required by interface
     * 
     */ 
    ~VehicleModelAccessor();

    //
    // Overriden interface functions
    //

    std::vector<VehicleState> predict(VehicleState initial_state,
      double timestep, double delta_t) override; 

    std::vector<VehicleState> predict(VehicleState initial_state,
      std::vector<VehicleModelControlInput> control_inputs, double timestep) override;
};