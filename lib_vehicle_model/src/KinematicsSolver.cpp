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

#include "KinematicsSolver.h"

/**
 * Cpp containing the implementation of KinematicsSolver
 */

double KinematicsSolver::solve(KinematicsProperty output_prop, KinematicsProperty unavailable_prop,
   double prop1, double prop2, double prop3) {
  switch(output_prop) {

    case INITIAL_VELOCITY:
      switch(output_prop) {
        case FINAL_VELOCITY:
          //a,d,t
          break;
        case ACCELERATION:
        //v_f,d,t
          break;
        case DISTANCE:
        //v_f,a,t
          break;
        case TIME:
        //v_f,a,d
          break;
      }
      break;

    case FINAL_VELOCITY:
      switch(output_prop) {
        case INITIAL_VELOCITY:
        //a,d,t
          break;
        case ACCELERATION:
        //v_i,d,t
          break;
        case DISTANCE:
        //v_i,a,t
          break;
        case TIME:
        //v_i,a,d
          break;
      }
      break;

    case ACCELERATION:
      switch(output_prop) {
        case INITIAL_VELOCITY:
        //v_f,d,t
          break;
        case FINAL_VELOCITY:
        //v_i,d,t
          break;
        case DISTANCE:
        //v_i,v_f,t
          break;
        case TIME:
        //v_i,v_f,d
          break;
      }
      break;

    case DISTANCE:
      switch(output_prop) {
        case INITIAL_VELOCITY:
        //v_f,a,t
          break;
        case FINAL_VELOCITY:
        //v_i,a,t
          break;
        case ACCELERATION:
        //v_i,v_f,t
          break;
        case TIME:
        //v_i,v_f,a
          break;
      }
      break;
      
    case TIME:
      switch(output_prop) {
        case INITIAL_VELOCITY:
        //v_f,a,d
          break;
        case FINAL_VELOCITY:
        //v_i,a,d
          break;
        case ACCELERATION:
        //v_i,v_f,d
          break;
        case DISTANCE:
        //v_i,v_f,a
          break;
      }
      break;
  }

  return 0;// TODO remove or add defaults
}
