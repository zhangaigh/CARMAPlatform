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
 * NOTE THIS IS NOT A RUNTIME USABLE CLASS
 * @class MockExternalVehicleMotionModel
 * @brief Mock external vehicle model to allow library to compile. TODO Remove after a real model is created
 * 
 */
class MockExternalVehicleMotionModel 
{
  public:
    
    std::vector<cav_msgs::VehicleState> predict(cav_msgs::VehicleState initial_state,
      double timestep, double delta_t) {
      std::vector<cav_msgs::VehicleState> fake_output;
      fake_output.push_back(initial_state);

      return fake_output;  
    }

    void setParameterServer(ParameterServer parameter_server) {
      // Do nothing
    }
};