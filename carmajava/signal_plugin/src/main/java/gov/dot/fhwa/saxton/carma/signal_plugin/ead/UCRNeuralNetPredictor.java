/*
 * Copyright (C) 2018 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

import java.util.LinkedList;
import java.util.List;

import cav_msgs.RoadwayObstacle;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;

/**
 * The SimpleNCVMotionPredictor provides an implementation of IMotionPredictor using simple linear regression to predict future detected vehicle motions.
 * The returned list of RoutePointStamped objects can be used to check conflicts with the CARMA conflict detection system
 * 
 * TODO
 */
public class UCRNeuralNetPredictor implements IMotionPredictor {

   @Override
  public List<RoutePointStamped> predictMotion(String objId, List<RoadwayObstacle> objTrajectory, double distanceStep, double timeDuration) {

    List<RoutePointStamped> projection = new LinkedList<>();

    // Return an empty list if provided with less than 2 points
    if (objTrajectory.size() < 2) {
      return projection; 
    }

    return projection;
  }
}