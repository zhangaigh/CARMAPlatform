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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.SignalPhase;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import static gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.Constants.MPS_TO_MPH;

/**
 * Calculates the viable neighbors for a node in the fine grid solution tree for the EAD model.
 */
public class FinePathNeighbors extends NeighborBase {

    protected double                        acceptableStopDist_; //max dist before bar it's acceptable to stop, m
    protected double                        debugThreshold_;//node distance beyond which we will turn on debug logging
    protected double                        responseLag_; //vehicle dynamic response lag, sec
    protected double                        allowableSpeedRegion_;
    protected List<Node>                    path_;
    protected static ILogger                log_ = LoggerManager.getLogger(FinePathNeighbors.class);
    protected static double                 FLOATING_POINT_EPSILON = 0.1;


    public FinePathNeighbors() {
        history_ = new ArrayList<>();

        //get config parameters
        IGlidepathAppConfig config = GlidepathApplicationContext.getInstance().getAppConfig();
        maxAccel_ = config.getDoubleDefaultValue("defaultAccel", 2.0);
        speedLimit_ = config.getMaximumSpeed(0.0) / MPS_TO_MPH;
        crawlingSpeed_ = config.getDoubleDefaultValue("crawlingSpeed", 5.0) / MPS_TO_MPH;
        acceptableStopDist_ = config.getDoubleDefaultValue("ead.acceptableStopDistance", 6.0);
        timeBuffer_ = config.getDoubleDefaultValue("ead.timebuffer", 4.0);
        debugThreshold_ = config.getDoubleDefaultValue("ead.debugThreshold", -1.0);
        responseLag_ = config.getDoubleDefaultValue("ead.response.lag", 1.9);
        allowableSpeedRegion_ = config.getDoubleDefaultValue("ead.allowableSpeedRegion", 5.0);
    }


    @Override
    public void initialize(List<IntersectionData> intersections, int numIntersections, double timeIncrement,
                           double speedIncrement) {

        log_.info("EAD", "initialize called with timeInc = " + timeIncrement + ", speedInc = " + speedIncrement);
        // Set the acceptable stop distance to at least half the distance increment
        acceptableStopDist_ = Math.max(1.1 * 2.0 * timeIncrement * speedIncrement, acceptableStopDist_); 
        log_.info("EAD", "Using acceptable stop distance of: " + acceptableStopDist_);
        super.initialize(intersections, numIntersections, timeIncrement, speedIncrement);
    }

    @Override
    public List<Node> neighbors(Node node) {
        List<Node> neighbors = new ArrayList<>();

        double curTime = node.getTimeAsDouble();
        double curDist = node.getDistanceAsDouble();
        double curSpeed = node.getSpeedAsDouble();
        
        //if this node will inevitably result in signal violation, return no neighbors
        double timeToStop = curSpeed / maxAccel_;
        double distToStop = timeToStop * curSpeed * 0.5;
        if(signalViolation(curDist, curDist + distToStop, curTime, timeToStop + curTime)) {
            log_.debug("PLAN", "this node has no neighbor because of signal violation: " + node);
            return neighbors;
        }

        double variableTimeInc = Math.max(timeInc_, responseLag_);
        double newTime = curTime + variableTimeInc;

        List<Double> speeds = getViableSpeeds(node, variableTimeInc);
        
        //loop on the reachable speed increments
        for(double v : speeds) {
            double deltaD = variableTimeInc * (curSpeed + v) * 0.5;
            double newDist = curDist + deltaD;
            if(!signalViolation(curDist, newDist, curTime, newTime) && isInAllowableSpeedRegion(v, newDist)) {
                neighbors.add(new Node(newDist, newTime, v));
            } else {
                log_.debug("PLAN", "Remove candidate speed due to signal violation: " + v);
            }
        }
        log_.debug("PLAN", "Generating neighbors of size " + neighbors.size());
        return neighbors;
    }
    
    public void setCoarsePlan(List<Node> path) {
        path_ = path;
    }

    ////////////////////

    private boolean isInAllowableSpeedRegion(double speed, double dist) {
        if(path_ == null || path_.size() == 0) {
            return true;
        }
        for(int i = 0; i < path_.size(); i++) {
            if(dist < path_.get(i).getDistanceAsDouble()) {
                if(i == path_.size() - 1) {
                    return true;
                } else {
                    continue;
                }
            } else {
                if(i == 0) {
                    return true;
                } else {
                    Node previousNode = path_.get(i - 1), nextNode = path_.get(i);
                    double deltaD = nextNode.getDistanceAsDouble() - previousNode.getDistanceAsDouble();
                    double deltaV = nextNode.getSpeedAsDouble() - previousNode.getSpeedAsDouble();
                    double coarsePlanSpeed = (((dist - previousNode.getDistanceAsDouble()) / deltaD) * deltaV) + previousNode.getSpeedAsDouble();
                    return Math.abs(coarsePlanSpeed - speed) <= allowableSpeedRegion_;
                }
            }
        }
        return false;
    }
    
    private List<Double> getViableSpeeds(Node node, double variableTimeInc) {
        double curTime = node.getTimeAsDouble();
        double curDist = node.getDistanceAsDouble();
        double curSpeed = node.getSpeedAsDouble();

        List<Double> speeds = new ArrayList<>();

        //get candidate upper & lower speeds based on acceleration limit, time increment and speed limit
        // we will create nodes at regular increments between these
        double minSpeed = Math.max(curSpeed - maxAccel_*variableTimeInc, 0.0);
        double maxSpeed = Math.min(curSpeed + maxAccel_*variableTimeInc, speedLimit_);
        log_.debug("PLAN", "Generating neighbors of node " + node.toString() + ". Initial minSpeed = " + minSpeed + ", maxSpeed = " + maxSpeed);

        int currentInt = currentIntersectionIndex(curDist);
        double dtsb = distToIntersection(currentInt, curDist);
        // If the current node is a zero speed node within the stop distance then a stop has occurred, any future node should be 0.0 unless the phase will be green
        if (dtsb <= acceptableStopDist_ && currentInt >= 0) {
            boolean nextTimeIsGreen = phaseAtTime(currentInt, curTime + variableTimeInc).phase.equals(SignalPhase.GREEN);
            if (curSpeed < FLOATING_POINT_EPSILON && !nextTimeIsGreen) {
                speeds.add(0.0);
                return speeds;
            }
        }

        //we only allow small speed around acceptableStopDist_ and will be capped to 0.0
        if(dtsb <= acceptableStopDist_ && minSpeed < FLOATING_POINT_EPSILON) {
            speeds.add(0.0);
        } else {
            speeds.add(Math.max(minSpeed, crawlingSpeed_));
        }

        double newSpeed = curSpeed;
        //add current speed if it is larger than crawling speed
        if(curSpeed > crawlingSpeed_) {
            speeds.add(curSpeed);
        }
        //decrement from the current speed minus speedInc until the minSpeed
        newSpeed = curSpeed - speedInc_;
        while(newSpeed > minSpeed) {
            if (newSpeed > crawlingSpeed_) {
                speeds.add(newSpeed);
            }
            newSpeed -= speedInc_;
        }

        //increment from the current speed plus speedInc until the maxSpeed limit
        newSpeed = curSpeed + speedInc_;
        while(newSpeed < maxSpeed) {
            if (newSpeed > crawlingSpeed_) {
                speeds.add(newSpeed);
            }
            newSpeed += speedInc_; 
        }
        speeds.add(maxSpeed);

        return speeds;
    }

    /**
     * Determines if the vehicle would violate signal laws when travelling from the start to final conditions specified
     * @param startDist - starting distance downtrack of plan start, m
     * @param endDist - final distance downtrack of plan start, m
     * @param startTime - starting time since beginning of plan, sec
     * @param endTime - final time since start of plan, sec
     * @return true if a violation would occur (runs a red light) while moving from start to end
     */
    private boolean signalViolation(double startDist, double endDist, double startTime, double endTime) {
        //log_.debug("EAD", "Entering signalViolation: startDist = " + startDist + ", endDist = "
        //            + endDist + ", startTime = " + startTime + ", endTime = " + endTime);

        //determine which intersection we are approaching [currentIntersectionIndex]
        int currentInt = currentIntersectionIndex(startDist);
        if (currentInt == -1) {
            return false;
        }

        //if the travel from start to end is not going to cross the stop bar then
        double distToInt = distToIntersection(currentInt, endDist);
        if (distToInt > 0.0) {
            return false;
        }

        //figure out exactly when we will cross the stop bar
        // Note: distToIntersection here will give the distance from start of plan to the stop bar, which is
        // the location of interest where we will cross; use this since all other distances are relative to plan start
        double interpFactor = (distToIntersection(currentInt, 0.0) - startDist) / (endDist - startDist);
        double crossingTime = startTime + interpFactor*(endTime - startTime);
        //log_.debug("EAD", "Stop bar will be crossed: interpFactor = " + interpFactor
        //            + ", crossingTime = " + crossingTime);

        //to account for uncertainties in the vehicle's dynamic response to speed command changes, we need
        // a little wiggle room, so don't want to cross the bar just as signal is changing color; we want
        // to avoid red at crossing time +/- the specified buffer (use a smaller time buffer to make fine plan easier)
        boolean redIfEarly = phaseAtTime(currentInt, crossingTime - (timeBuffer_ * 0.25)).phase.equals(SignalPhase.RED);
        boolean redIfLate  = phaseAtTime(currentInt, crossingTime + (timeBuffer_ * 0.25)).phase.equals(SignalPhase.RED);
        return redIfEarly || redIfLate;
    }


    private void logDebug(double distance, String msg) {
        if (debugThreshold_ >= 0.0  &&  distance >= debugThreshold_) {
            log_.debug("PLAN", msg);
        }
    }
}
