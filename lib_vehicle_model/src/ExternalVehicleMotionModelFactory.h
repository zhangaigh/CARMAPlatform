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

#include "ParameterServer.h"
#include "VehicleClass.h"
#include "ExternalVehicleMotionModel.h"

/**
 * @class ExternalVehicleMotionModelFactory
 * @brief A factory which constructs an ExternalVehicleMotionModel to match the requested vehicle class. 
 * 
 */
class ExternalVehicleMotionModelFactory 
{
  private:
    ParameterServer param_server_;

  public:
    /**
     * @brief Constructor
     * 
     * @param nh A reference to a ParameterServer which can be used to access parameters needed by vehicle motion models
     */ 
    ExternalVehicleMotionModelFactory(ParameterServer& parameter_server);

    /**
     * @brief Constructs an ExternalVehicleMotionModel for the specified VehicleClass
     * 
     * @param vehicle_class The vehicle class which the constructed motion model will apply to
     * 
     * @return An ExternalVehicleMotionModel which is designed to predict motion for vehicles of the same classification as vehicle_class
     */
    ExternalVehicleMotionModel build(VehicleClass vehicle_class); 
};