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

#include <iostream>
#include <sstream>

namespace lib_vehicle_model {

  /**
   * @enum KinematicsProperty
   * @brief An enumeration which defines the possible properties in a basic kinematic equation using constant acceleration
   * 
   * For example d = 0.5*at^2 + v_0*t contains the properties distance, acceleration, initial velocity, and time, but not final velocity
   * 
   */
  enum KinematicsProperty 
  {
    INITIAL_VELOCITY,
    FINAL_VELOCITY,
    ACCELERATION,
    DISTANCE,
    TIME
  };

  /**
   * Overload of << operation so enum objects will output as strings in print functions
   * 
   */ 
  std::ostream& operator<<( std::ostream& os, const KinematicsProperty& prop )
  {
    switch( prop )
    {
        case INITIAL_VELOCITY: os << "INITIAL_VELOCITY"; break;
        case FINAL_VELOCITY: os << "FINAL_VELOCITY"; break;
        case ACCELERATION: os << "ACCELERATION"; break;
        case DISTANCE: os << "DISTANCE"; break;
        case TIME: os << "TIME"; break;
        default: os << "ERROR: UNKNOWN TYPE"; break;
    }
  }
}