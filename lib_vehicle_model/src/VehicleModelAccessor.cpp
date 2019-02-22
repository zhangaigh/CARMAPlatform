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

#include "VehicleModelAccessor.h"



/**
 * Cpp containing the implementation of VehicleModelAccessor
 */

VehicleModelAccessor::VehicleModelAccessor(ParameterServer& parameter_server): param_server_(parameter_server) {
  // TODO load params here
}

VehicleModelAccessor::~VehicleModelAccessor() {
  // TODO safely close libraries here
}

std::vector<cav_msgs::VehicleState> VehicleModelAccessor::predict(cav_msgs::VehicleState initial_state,
  double timestep, double delta_t) {
    // TODO pass to loaded library here
  }

std::vector<cav_msgs::VehicleState> VehicleModelAccessor::predict(cav_msgs::VehicleState initial_state,
  std::vector<cav_msgs::VehicleState> control_inputs, double timestep) {
    // TODO pass to loaded library here
  }

std::vector<cav_msgs::VehicleState> VehicleModelAccessor::predict(cav_msgs::VehicleState initial_state,
  std::vector<cav_msgs::Maneuvers> maneuvers, double timestep) {
    // TODO pass to loaded library here
  }
