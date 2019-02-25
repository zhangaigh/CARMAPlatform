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
  // Load Parameters
  bool all_params;
  all_params = all_params && param_server_.getParam("vehicle_model_lib_path", vehicle_model_lib_path_);
  all_params = all_params && param_server_.getParam("max_forward_speed", max_forward_speed_);
  all_params = all_params && param_server_.getParam("max_reverse_speed", max_reverse_speed_);
  all_params = all_params && param_server_.getParam("forward_acceleration_limit", forward_acceleration_limit_);
  all_params = all_params && param_server_.getParam("forward_deceleration_limit", forward_deceleration_limit_);
  all_params = all_params && param_server_.getParam("reverse_acceleration_limit", reverse_acceleration_limit_);
  all_params = all_params && param_server_.getParam("reverse_deceleration_limit", reverse_deceleration_limit_);
  all_params = all_params && param_server_.getParam("max_steering_angle", max_steering_angle_);
  all_params = all_params && param_server_.getParam("min_steering_angle", min_steering_angle_);
  all_params = all_params && param_server_.getParam("max_steering_angle_rate", max_steering_angle_rate_);
  all_params = all_params && param_server_.getParam("max_trailer_angle", max_trailer_angle_);
  all_params = all_params && param_server_.getParam("min_trailer_angle", min_trailer_angle_);

  // Check if all the required parameters could be loaded
  if (!all_params) {
    // TODO throw exception
  }

  // Load the vehicle model to be used
  loadModel();
  vehicle_model_->setParameterServer(param_server_);

  // Then we need a create function with returns a pointer to our library object
  // In our cpp file there should be a create and destroy function which is not in the class name space. This will return an instance of our model
}

VehicleModelAccessor::loadModel() {
    void *lib_handle;
    lib_handle = dlopen(vehicle_model_lib_path_, RTLD_NOW);

    if (!lib_handle)
    {
      ROS_FATAL_STREAM("Failed to open vehicle model shared library at " << vehicle_model_lib_path_ << " Reported Error: " << dlerror());
    }
 
    create_fnc_ = (create_fnc_ptr)dlsym(lib_handle,"create");
    destroy_fnc_ = (destroy_fnc_ptr)dlsym(lib_handle,"destroy");

    if (!create_fnc_)
    {
      ROS_FATAL_STREAM("Failed to find pointer to vehicle model shared library create function " << " Reported Error: " << dlerror());
    }

    if (!destroy_fnc_)
    {
      ROS_FATAL_STREAM("Failed to find pointer to vehicle model shared library destroy function " << " Reported Error: " << dlerror());
    }

    // Set the vehicle model to the object returned by create_fnc
    // Pass in the destroy_fnc as the deletor. 
    vehicle_model_.reset(create_fnc_(), destroy_fnc_);
}

VehicleModelAccessor::~VehicleModelAccessor() {
  // No need to do anything. When the smart pointer goes out of scope the destroy_fnc will be called for the library
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
