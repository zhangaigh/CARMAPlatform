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
#include <cav_msgs/VehicleState.h>

/**
 * @class ExternalVehicleMotionModel
 * @brief An interface which defines the functions needed to predict external vehicle motion. 
 * 
 * Unlike the VehicleMotionModel interface this predict function does not depend on control inputs and do not require that all elements of the vehicle state vector be filled in.
 */
class ExternalVehicleMotionModel 
{
  public:
    /**
     * @brief Predict vehicle motion assuming no change in control input
     * 
     * @param initial_state The starting state of the vehicle
     * @param timestep The time increment between returned traversed states
     * @param delta_t The time to project the motion forward for
     * 
     * @return A list of traversed states seperated by the timestep
     * 
     * NOTE: Not all elements of the input vehicle state a required to be populated. 
     * 
     */
    virtual std::vector<cav_msgs::VehicleState> predict(cav_msgs::VehicleState initial_state,
      double timestep, double delta_t) = 0; // Defined as pure virtual function

    /**
     * @brief Set the parameter server which will be used by vehicle models
     * 
     * @param parameter_server The parameter server to use when a vehicle model is loading parameters
     * 
     */
    virtual void setParameterServer(ParameterServer parameter_server) = 0; // Defined as pure virtual function
};