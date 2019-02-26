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

VehicleModelAccessor::VehicleModelAccessor(std::shared_ptr<ParameterServer> parameter_server) {

  param_server_ = parameter_server;

  // Load Parameters
  bool all_params;
  all_params = all_params && param_server_->getParam("vehicle_model_lib_path", vehicle_model_lib_path_);
  all_params = all_params && param_server_->getParam("max_forward_speed", max_forward_speed_);
  all_params = all_params && param_server_->getParam("max_reverse_speed", max_reverse_speed_);
  all_params = all_params && param_server_->getParam("forward_acceleration_limit", forward_acceleration_limit_);
  all_params = all_params && param_server_->getParam("forward_deceleration_limit", forward_deceleration_limit_);
  all_params = all_params && param_server_->getParam("reverse_acceleration_limit", reverse_acceleration_limit_);
  all_params = all_params && param_server_->getParam("reverse_deceleration_limit", reverse_deceleration_limit_);
  all_params = all_params && param_server_->getParam("max_steering_angle", max_steering_angle_);
  all_params = all_params && param_server_->getParam("min_steering_angle", min_steering_angle_);
  all_params = all_params && param_server_->getParam("max_steering_angle_rate", max_steering_angle_rate_);
  all_params = all_params && param_server_->getParam("max_trailer_angle", max_trailer_angle_);
  all_params = all_params && param_server_->getParam("min_trailer_angle", min_trailer_angle_);

  // Check if all the required parameters could be loaded
  if (!all_params) {
    throw std::invalid_argument("One of the required parameters could not be found or read");
  }

  // Load the vehicle model to be used
  loadModel();
  vehicle_model_->setParameterServer(param_server_);
}

VehicleModelAccessor::~VehicleModelAccessor() {};

void VehicleModelAccessor::loadModel() {
  // Load library from path
  void *lib_handle;
  lib_handle = dlopen(vehicle_model_lib_path_.c_str(), RTLD_NOW);

  // Check if load successfull
  if (!lib_handle)
  {
    std::string errorLog(dlerror());
    throw std::invalid_argument("Failed to open vehicle model shared library at " + vehicle_model_lib_path_ + " Reported Error: " + errorLog);
  }

  // Get pointers to the create and destroy functions
  create_fnc_ = (create_fnc_ptr)dlsym(lib_handle,"create");
  destroy_fnc_ = (destroy_fnc_ptr)dlsym(lib_handle,"destroy");

  // Check if create and destroy functions could be found
  if (!create_fnc_)
  {
    std::string errorLog(dlerror());
    throw std::invalid_argument("Failed to find pointer to vehicle model shared library create function Reported Error: " + errorLog);
  }

  if (!destroy_fnc_)
  {
    std::string errorLog(dlerror());
    throw std::invalid_argument("Failed to find pointer to vehicle model shared library destroy function Reported Error: " + errorLog);
  }

  // Set the vehicle model to the object returned by create_fnc
  // Pass in the destroy_fnc as the smart pointer deletor
  vehicle_model_.reset(create_fnc_(), destroy_fnc_);
}

std::vector<VehicleState> VehicleModelAccessor::predict(VehicleState initial_state,
  double timestep, double delta_t) {
    // TODO add constraint checks
    std::vector<VehicleState> vec;
    return vec;
    //return vehicle_model_->predict(initial_state, timestep, delta_t);
  }

std::vector<VehicleState> VehicleModelAccessor::predict(VehicleState initial_state,
  std::vector<VehicleModelControlInput> control_inputs, double timestep) {
    // TODO add constraint checks
    std::vector<VehicleState> vec;
    return vec;
   // return vehicle_model_->predict(initial_state, control_inputs, timestep);
  }
