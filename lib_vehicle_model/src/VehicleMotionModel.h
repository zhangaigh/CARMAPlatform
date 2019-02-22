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

#include <cav_msgs/VehicleState.h>
#include <cav_msgs/VehicleControlInput.h>
#include <cav_msgs/Maneuvers.h> // This may be provided as an actual class rather than ROS message
#include "ParameterServer.h"



/**
 * @class VehicleMotionModel
 * @brief Interfaces which all implemented vehicle models must adhere to. 
 * 
 * This interface extends the VehicleMotionPredictor by adding a setParameterServer function. 
 */
class VehicleMotionModel: public virtual VehicleMotionPredictor
{
  public:
    /**
     * @brief Set the parameter server which will be used by vehicle models
     * 
     * @param parameter_server The parameter server to use when a vehicle model is loading parameters
     * 
     */
    virtual void setParameterServer(ParameterServer parameter_server) = 0; // Defined as pure virtual function
};