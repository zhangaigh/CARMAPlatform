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
#include <cav_msgs/VehicleControlInput.h>
#include <cav_msgs/Maneuvers.h> // This may be provided as an actual class rather than ROS message
#include "VehicleMotionPredictor.h"



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
    ParameterServer param_server_;

  public:

    /**
     * @brief Constructor 
     * 
     * @param parameter_server A reference to the parameter server which vehicle models will use to load parameters
     * 
     */ 
    VehicleModelAccessor(ParameterServer& parameter_server);

    /**
     * @brief Destructor ensures that any loaded vehicle model libraries are properly shutdown when this object is destroyed 
     */ 
    ~VehicleModelAccessor();

    std::vector<cav_msgs::VehicleState> predict(cav_msgs::VehicleState initial_state,
      double timestep, double delta_t) override; 

    std::vector<cav_msgs::VehicleState> predict(cav_msgs::VehicleState initial_state,
      std::vector<cav_msgs::VehicleState> control_inputs, double timestep) override;

    std::vector<cav_msgs::VehicleState> predict(cav_msgs::VehicleState initial_state,
      std::vector<cav_msgs::Maneuvers> maneuvers, double timestep) override;
};