package gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.SignalPhase;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.INodeCollisionChecker;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

import java.util.ArrayList;
import java.util.List;

import static gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.Constants.MPS_TO_MPH;

/**
 * Calculates the viable neighbors for a node in the fine grid solution tree for the EAD model.
 */
public class FinePathNeighbors extends NeighborBase {

    protected double                        stoppingLookAhead_; // time, sec, to test for imminent red violation
    protected double                        acceptableStopDist_; //max dist before bar it's acceptable to stop, m
    protected double                        debugThreshold_;//node distance beyond which we will turn on debug logging
    protected static ILogger                log_ = LoggerManager.getLogger(FinePathNeighbors.class);


    public FinePathNeighbors() {
        history_ = new ArrayList<>();

        //get config parameters
        IGlidepathAppConfig config = GlidepathApplicationContext.getInstance().getAppConfig();
        maxAccel_ = config.getDoubleDefaultValue("defaultAccel", 2.0);
        speedLimit_ = config.getDoubleDefaultValue("maximumSpeed", 30.0) / MPS_TO_MPH;
        crawlingSpeed_ = config.getDoubleDefaultValue("crawlingSpeed", 5.0) / MPS_TO_MPH;
        acceptableStopDist_ = config.getDoubleDefaultValue("ead.acceptableStopDistance", 6.0);
        stoppingLookAhead_ = 5.0; //default
        timeBuffer_ = config.getDoubleDefaultValue("ead.timebuffer", 4.0);
        debugThreshold_ = config.getDoubleDefaultValue("ead.debugThreshold", -1.0);
    }


    @Override
    public void initialize(List<IntersectionData> intersections, int numIntersections, double timeIncrement,
                           double speedIncrement, INodeCollisionChecker collisionChecker) {

        log_.debug("EAD", "initialize called with timeInc = " + timeIncrement + ", speedInc = " + speedIncrement);
        super.initialize(intersections, numIntersections, timeIncrement, speedIncrement, collisionChecker);

        //ensure we will be thinking about stopping at least 2 nodes prior to the stop bar
        stoppingLookAhead_ = Math.max(stoppingLookAhead_, 2.01*timeIncrement);
    }


    @Override
    public List<Node> neighbors(Node node) {
        List<Node> neighbors = new ArrayList<>();

        //all valid neighbors will have their time one time increment larger than the input node's
        double curTime = node.getTimeAsDouble();
        double curDist = node.getDistanceAsDouble();
        double curSpeed = node.getSpeedAsDouble();
        double newTime = curTime + timeInc_;

        //determine viable speeds for neighbors, based on accel limit and other limitations
        // Lower limit is arbitrarily configured as crawling speed to avoid control problems at really low
        // speeds. We are only allowed to set target speeds below this value if we are coming to a stop or starting
        // out from a stop. Nodes with a speed of exactly zero are normal and acceptable, however (sitting
        // at a red light).

        //get candidate upper & lower speeds based on acceleration limits; we will create nodes at regular increments
        // between these
        double minSpeed = Math.max(curSpeed - maxAccel_*timeInc_, 0.0);
        double maxSpeed = Math.min(curSpeed + maxAccel_*timeInc_, speedLimit_);
        logDebug(curDist, "Generating neighbors of node " + node.toString() +
                    ". Initial minSpeed = " + minSpeed + ", maxSpeed = " + maxSpeed);

        List<Double> speeds = new ArrayList<>();
        boolean stopping = false;
        boolean starting = false;

        //if current speed > crawling, allow it to be a candidate for future nodes
        if (curSpeed >= crawlingSpeed_) {
            speeds.add(curSpeed);
        }

        //if we are at risk of running a red light in the next few sec or are moving very slowly in the vicinity
        // of one (indicates we are on a stopping trajectory) then
        double endDist = curDist + stoppingLookAhead_*Math.max(curSpeed, crawlingSpeed_); //worst case
        boolean violation = signalViolation(curDist, endDist, curTime, curTime + stoppingLookAhead_);
        double dtsb = distToIntersection(currentIntersectionIndex(curDist), curDist);
        if (violation  ||  (curSpeed < crawlingSpeed_ && dtsb < 3.0*acceptableStopDist_)) {
            //indicate we need to be stopping
            stopping = true;
            logDebug(curDist, "We are presumably stopping. curSpeed = " + curSpeed);

        //else if current speed is less than crawling speed then
        }else if (curSpeed < crawlingSpeed_) {
            //indicate we are starting up
            starting = true;
            logDebug(curDist, "We are presumably starting up. curSpeed = " + curSpeed);
        }

        //if we are not in the process of stopping then
        if (!stopping) {
            //if we are starting up then
            if (starting) {
                //limit the minimum speed to a hair above current (force an acceleration)
                minSpeed = curSpeed + 0.1;
            }else {
                //we are at least at crawling speed, so limit the minimum speed to crawling
                minSpeed = Math.max(minSpeed, crawlingSpeed_);
            }

            //find viable speeds above current speed
            double speed = curSpeed + speedInc_;
            double prevMaxSpeed = curSpeed;
            while (speed <= maxSpeed) {
                speeds.add(speed);
                prevMaxSpeed = speed;
                speed += speedInc_;
            }

            //if acceleration allows us to reach speed limit, ensure it is one of the speeds being evaluated
            // (subtract 0.01 to account for possible round-off error when matching maxSpeed; subtract 0.25
            // on the right side to avoid checking two speeds that are really close together)
            if (maxSpeed >= speedLimit_ - 0.01  &&  prevMaxSpeed < speedLimit_ - 0.25) {
                speeds.add(speedLimit_);
            }
        }

        //if we are not starting up then
        if (!starting) {
            //look at all lower speeds
            double speed = curSpeed - speedInc_;
            double prevMinSpeed = curSpeed;
            while (speed >= minSpeed) {
                speeds.add(speed);
                prevMinSpeed = speed;
                speed -= speedInc_;
            }

            //if we are allowed to go to full stop then ensure it is one of the speeds being evaluated
            if (minSpeed == 0.0  &&  (prevMinSpeed > 0.0  ||  curSpeed == 0.0)) {
                speeds.add(0.0);
            }
        }

        //loop on the reachable speed increments
        if (speeds.size() > 0) {
            for (double s : speeds) {
                //compute the distance to be travelled going from the current speed to that new speed
                double newDist = curDist + 0.5 * (curSpeed + s) * timeInc_;

                //if the new downtrack distance is  going to violate a red signal then don't allow this to be a neighbor
                if (signalViolation(curDist, newDist, curTime, newTime)) {
                    logDebug(curDist, "Speed " + s + " will give a signal violation. Throwing it away.");
                    continue;
                }

                //if the new speed is 0 and we are somewhat close to the stop bar (appear to be stopping at red), but
                // uncomfortably far away for stopping at a light then ignore this possible neighbor
                // (we need to only test when dtsb is kinda close to stop bar, because we may be legitimately
                // stopped farther away, waiting to start an experiment)
                dtsb = distToIntersection(currentIntersectionIndex(curDist), curDist);
                if (s < 0.1 && dtsb > acceptableStopDist_ && dtsb < 6.0 * acceptableStopDist_) {
                    logDebug(curDist, "Speed " + s + " will stop too soon. Throwing it away.");
                    continue;
                }

                //add a node to the neighbor list to represent this new situation
                Node newNode = new Node(newDist, newTime, s);
                neighbors.add(newNode);



                //TODO:  this is for debugging only - need to remove before releasing code!
                if (node.getTime() == newNode.getTime()) {
                    log_.debug("PATH", "*** Created a naighbor with same time as parent:");
                    log_.debug("PATH", "***    parent = " + node.toString());
                    log_.debug("PATH", "***    neighb = " + newNode.toString());
                }
            }
        }
        log_.debug("PATH", "found " + neighbors.size() + " candidate neighbors.");
        
        //remove all conflicting nodes with NCV from the candidate neighbor node list
        List<Node> res = this.removeConflictingNode(node, neighbors);
        log_.debug("PATH", "returning " + res.size() + " neighbors after collision checking in fine plan.");

        return res;
    }


    ////////////////////


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
        // to avoid red at crossing time +/- the specified buffer
        boolean redIfEarly = phaseAtTime(currentInt, crossingTime - timeBuffer_).phase.equals(SignalPhase.RED);
        boolean redIfLate  = phaseAtTime(currentInt, crossingTime + timeBuffer_).phase.equals(SignalPhase.RED);
        return redIfEarly || redIfLate;
    }


    private void logDebug(double distance, String msg) {
        if (debugThreshold_ >= 0.0  &&  distance >= debugThreshold_) {
            log_.debug("PLAN", msg);
        }
    }
}
