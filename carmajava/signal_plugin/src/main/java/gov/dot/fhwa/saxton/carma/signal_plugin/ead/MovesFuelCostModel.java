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
package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.ICostModel;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

/**
 * Cost model for use in the trajectory tree solution. Computes a cost between neighboring nodes
 * based on an estimation of fuel cost to travel between those nodes.
 *
 * This version uses the cost model presented in the UCR white paper, Glidepath II MOVES Brief, dated Dec 2017.
 * Some assumptions are made in this implementation.
 * It assumes a flat roadway (no grade) and uniform acceleration between the given nodes.
 * It also assumes acceleration <= -2.0 corresponds to deceleration
 * It also assumes that unknown operating modes have a cost equal to the highest cost present in the provided tables
 */
public class MovesFuelCostModel implements ICostModel {

    // Goal evaluation variables
    private Node            goal = new Node(0, 0, 0);
    private Node            tolerances = null;

    // Cost calculation variables
    private final double rollingTermA;
    private final double rotatingTermB;
    private final double dragTermC;
    private final double vehicleMassInTons;
    private final double fixedMassFactor;
    private final Map<Integer,List<Double>> baseRateTable;
    private final int BASE_RATE_ENERGY_COL = 4;
    private final double ROAD_GRADE = 0.0; // By default we assume the road is flat. 0.0 in rad
    private final int EXPECTED_BASE_RATE_TABLE_LENGTH = 7; // Number of columns in the energy consumption table
    private final double SEC_PER_HR = 3600.0;
    private final double J_PER_KJ = 1000.0;
    private final double DEFAULT_PEAK_ENERGY_KJ; // The highest cost found in the energy consumption table

    private int numCosts = 0;
    protected static final ILogger log = LoggerManager.getLogger(MovesFuelCostModel.class);
    protected final double fuelNormalizationDenominator;
    protected final double timeNormalizationDenominator;
    protected final double heuristicWeight;

    protected final double percentCostForTime;
    protected final double percentCostForFuel;
    protected final double maxVelocity;
    protected final double maxAccel;

    /**
     * Builds the cost model object with several injected calculation parameters.
     * These parameters should come from the EPA "MOVES2010 Highway Vehicle Population and Activity Data",
     * in accordance with the MOVES Brief document provided by UCR.
     * 
     * @param rollingTermA - Term corresponding to vehicle roll. Units: kW - s/m
     * @param rotatingTermB - Term corresponding to vehicle rotation. Units: kW - s^2/m^2
     * @param dragTermC - Term corresponding to vehicle drag. Units: kW - s^3/m^3
     * @param vehicleMassInTons - Source vehicle mass. Units: metric tons
     * @param fixedMassFactor - Fixed mass factor. Units: metric tons
     * @param baseRateTablePath - Path to the csv file used to store the energy consumption parameters based on vehicle state
     * @param fuelNormalizationDenominator - Divides the calculated fuel cost to normalize it for a range ~0-1
     * @param timeNormalizationDenominator - Divides the calculated time cost to normalize it for a range ~0-1
     * @param heuristicWeight - Weight factor to apply to the calculated heuristic. When non-one A* will behave as Weighted A*
     * @param percentCostForTime - A factor (0-1) which will be multiplied by the time cost after normalization. (1 - percentCostForTime) will be multiplied by the fuel.
     * @param maxVelocity - The maximum velocity the vehicle can travel in m/s
     * 
     * @throws IOException - Exception thrown when the file specified by baseRateTablePath cannot be loaded properly
     */
    public MovesFuelCostModel(double rollingTermA, double rotatingTermB, double dragTermC, double vehicleMassInTons,
        double fixedMassFactor, String baseRateTablePath,
        double fuelNormalizationDenominator, double timeNormalizationDenominator,
        double heuristicWeight,
        double percentCostForTime,
        double maxVelocity,
        double maxAccel) throws IOException {
        //assign injected params
        this.rollingTermA = rollingTermA;
        this.rotatingTermB = rotatingTermB;
        this.dragTermC = dragTermC;
        this.vehicleMassInTons = vehicleMassInTons;
        this.fixedMassFactor = fixedMassFactor;
        this.baseRateTable = loadTable(baseRateTablePath); // Load the base rates table

        this.percentCostForTime = percentCostForTime;
        this.percentCostForFuel = 1.0 - percentCostForTime;
        this.maxVelocity = maxVelocity;
        this.maxAccel = maxAccel;
        // Find the highest energy cost in the table and store it for use when values fall outside table scope
        double maxValue = 0; 
        for(Entry<Integer, List<Double>> entry: this.baseRateTable.entrySet()) {
            if (entry.getValue().get(BASE_RATE_ENERGY_COL) > maxValue) {
                maxValue = entry.getValue().get(BASE_RATE_ENERGY_COL);
            }
        }

        this.timeNormalizationDenominator = timeNormalizationDenominator;
        this.fuelNormalizationDenominator = fuelNormalizationDenominator;
        this.heuristicWeight = heuristicWeight;
        
        this.DEFAULT_PEAK_ENERGY_KJ = maxValue;
    }

    /**
     * Helper function to load a csv file containing the host vehicle energy consumption parameters based on vehicle state
     * 
     * @param baseRateTablePath - Path to the csv file used to store the energy consumption parameters based on vehicle state
     * @throws IOException - Exception thrown when the file specified by baseRateTablePath cannot be loaded properly
     * 
     * @return A mapping of operating mode id with energy consumption parameters 
     */
    private Map<Integer,List<Double>> loadTable(String baseRateTablePath) throws IOException {
        String line = "";
        String delimiter = ",";
        boolean firstLine = true;
        Map<Integer,List<Double>> map = new HashMap<>();

        try (BufferedReader br = new BufferedReader(new FileReader(baseRateTablePath))) {

            while ((line = br.readLine()) != null) {

                if (firstLine) {
                    firstLine = false;
                    continue;
                }
                String[] data = line.split(delimiter);

                if (data.length != EXPECTED_BASE_RATE_TABLE_LENGTH) {
                    throw new IOException("MOVES fuel cost model data file contained " + data.length + " columns but expected " + EXPECTED_BASE_RATE_TABLE_LENGTH);
                }

                Integer key = Integer.parseInt(data[0]);

                List<Double> values = new ArrayList<>(EXPECTED_BASE_RATE_TABLE_LENGTH - 1);

                for (int i = 1; i < data.length; i++) {
                    values.add(Double.parseDouble(data[i]));
                }

                map.put(key, values);
            }

        } catch (IOException e) {
            throw e;
        }
        return map;
    }


    /**
     * Expresses the cost of moving from n1 to n2 in terms of energy consumed.
     * Output cost is in units of KJ
     * Assumes n2.distance >= n1.distance and n2.time > n1.time
     * Calculation based on UCR MOVES Brief
     * 
     * @param n1 First node
     * @param n2 Second node
     * 
     * @return energy cost, J, or very large number if inputs are incorrect
     */
    @Override
    public double cost(Node n1, Node n2) {

        //input sanity checks
        if (n2.getTime() <= n1.getTime()  ||  n2.getDistance() < n1.getDistance()  ||
                n1.getSpeed() < 0  ||  n2.getSpeed() < 0) {
            log.debug("EAD", "Cost computation invoked with invalid nodes:");
            log.debug("EAD", "    n1: " + n1.toString());
            log.debug("EAD", "    n2: " + n2.toString());
            log.debug("EAD", "    " + numCosts + " costs have been evaluated since goal defined or previous error.");
            numCosts = 0;
            return Double.MAX_VALUE;
        }else {
            ++numCosts;
        }

        // Get VSP
        final double dv = n2.getSpeedAsDouble() - n1.getSpeedAsDouble();
        final double avg_v = (n2.getSpeedAsDouble() + n1.getSpeedAsDouble()) / 2.0;
        final double dt = n2.getTimeAsDouble() -  n1.getTimeAsDouble(); // Change in time in seconds
        final double a = dv / dt; // We are using the acceleration to get from current speed to target speed
        final double VSP = getVSP(a,avg_v);

        // Determine Operating Mode from OpModeTable
        final int opMode = getModeConditional(VSP, avg_v, a);

        // Return the highest cost which would still be in the table when our result is in the undefined region
        // Additionally log the occurrence
        if (opMode == -1) {
            log.debug("EAD", "Invalid operating mode found for MOVES cost calculation. Using highest known cost in table: " + DEFAULT_PEAK_ENERGY_KJ + " KJ");
            log.debug("EAD", "    Node 1: " + n1.toString());
            log.debug("EAD", "    Node 2: " + n2.toString());
            return J_PER_KJ * ((DEFAULT_PEAK_ENERGY_KJ / SEC_PER_HR) * dt);
        }

        // Normalize Cost
        double normalizedFuelCost = getJFromOpMode(opMode, dt) / fuelNormalizationDenominator;
        double normalizedTimeCost = dt / timeNormalizationDenominator;

        //System.out.println("normalizedFuelCost: " + normalizedFuelCost);
        
        // Return the a linear combination of the normalized fuel cost in J and time cost in s. 
        return normalizedFuelCost * percentCostForFuel + normalizedTimeCost * percentCostForTime;
    }

    /**
     * Helper function to convert KJ/Hr to J/s
     * 
     * @param kjPerHr The KJ/Hr to convert
     * 
     * @return The converted J/s
     */
    protected double toJPerSec(double kjPerHr) {
      return J_PER_KJ * (kjPerHr / SEC_PER_HR);
    }

    /**
     * Helper function to calculate the energy usage in joules 
     * 
     * @param opMode The operational mode to lookup emissions data for
     * @param dt The change in time in seconds
     * 
     * @return The usage of energy in joules
     */
    protected double getJFromOpMode(int opMode, double dt) {
      final List<Double> baseRateList = baseRateTable.get(opMode);

      return toJPerSec(baseRateList.get(BASE_RATE_ENERGY_COL)) * dt;
    }

    /**
     * Helper function to calculate the Vehicle Specific Power based on ending speed and acceleration to required to reach that speed
     * 
     * @param acceleration The acceleration required to go from current speed to endSpeed
     * @param averageSpeed The average speed over the region of given acceleration
     * 
     * @return The vehicle specific power
     */
    protected double getVSP(double accelerationToEndSpeed, double averageSpeed) {
        // Get VSP
        final double A = rollingTermA;
        final double B = rotatingTermB;
        final double C = dragTermC;
        final double M = vehicleMassInTons;
        final double f = fixedMassFactor;
        final double g = 9.8; // Acceleration due to gravity = 9.8 m/s^2
        final double v = averageSpeed; // We are using the target node's velocity
        final double v_sqr = v*v;
        final double a = accelerationToEndSpeed; // a = dV / dt : We are using the acceleration to get from current speed to target speed
        final double theta = ROAD_GRADE; 

        // Calculate the VehicleSpecificPower (VSP)
        final double VSP = (A*v + B*v_sqr + C*v_sqr*v + M*v*(a + g * Math.sin(theta))) / f;

        return VSP;
    }

    /**
     * Helper function for identifying the operating mode of the host vehicle based on vehicle specific power (VSP), velocity, and acceleration
     * 
     * This is a reconstruction of the conditional table lookup required by the UCR MOVES brief
     */
    protected int getModeConditional(double VSP, double velocity, double acceleration) {
        // In the MOVES documentation this table is presented in mi/hr here it is converted to m/s to avoid conversions
        final double FIFTY_MPH_IN_MPS = 22.352;
        final double TWENTY_FIVE_MPH_IN_MPS = 11.176;
        final double ONE_MPH_IN_MPS = 0.44704;
        // NOTE: This acceleration check is a simplification of a <= -2 OR (a < -1.0 AND a_t-1 < -1.0 AND a_t-2 < -1.0)
        //       The simplification was made to avoid the need to get previous acceleration values into this function
        if (acceleration <= -1.0) {
            return 0;
        }
        if (velocity >= FIFTY_MPH_IN_MPS) {
          if (VSP < 6.0) {
            return 33;
          } else if (6<= VSP && VSP < 12) {
            return 35;
          } else if (12<= VSP && VSP < 18) {
            return 37;
          } else if (18 <= VSP && VSP < 24) {
            return 38;
          } else if (24 <= VSP && VSP < 30) {
            return 39;
          } else {
            return 40;
          }
    
        } else if (-ONE_MPH_IN_MPS <= velocity && velocity < ONE_MPH_IN_MPS) {
          return 1;
        } else if (TWENTY_FIVE_MPH_IN_MPS <= velocity && velocity < FIFTY_MPH_IN_MPS) {
          if (VSP < 0) {
            return 21;
          } else if (0 <= VSP && VSP < 3) {
            return 22;
          } else if (3 <= VSP && VSP < 6) {
            return 23;
          } else if (6 <= VSP && VSP < 9) {
            return 24;
          } else if (9 <= VSP && VSP < 12) {
            return 25;
          } else if (12 <= VSP && VSP < 18) {
            return 27;
          } else if (18 <= VSP && VSP < 24) {
            return 28;
          }  else if (24 <= VSP && VSP < 30) {
            return 29;
          } else {
            return 30;
          }
        } else if (0.0 <= velocity && velocity < TWENTY_FIVE_MPH_IN_MPS) {
          if (VSP < 0) {
            return 11;
          } else if (0 <= VSP && VSP < 3) {
            return 12;
          } else if (3 <= VSP && VSP < 6) {
            return 13;
          } else if (6 <= VSP && VSP < 9) {
            return 14;
          } else if (9 <= VSP && VSP < 12) {
            return 15;
          } else {
            return 16;
          }
        } else {
          return -1;
        }
    }


    /**
     * Heuristic is calculated by finding the fastest trajectory to the goal assuming no lights or other vehicles.
     * The fuel cost comes from J/S of fuel usage used to idle multiplied by the travel time. 
     * This result is combined with the travel time using the same linear combination as the cost model.
     * This ensures that the heuristic is always consistent and directly comparable with the actual cost
     */
    @Override
    public double heuristic(Node currentNode) {

    if (isGoal(currentNode)) {
      return 0.0;
    }
     //infinite cost if we are passed the goal but did not satisfy the goal condition
     final double goalDistance = goal.getDistanceAsDouble() + tolerances.getDistanceAsDouble();
     if (currentNode.getDistance() > goalDistance) {
      return Double.POSITIVE_INFINITY;
    }

    /**
     * Minimum cost to goal will be fuel use while idle for the minimum possible travel time to the goal
     * The values will be normalized and weighted to mirror the cost function
     * This is guaranteed to by optimistic 
     */
    final double distanceToGoal = goalDistance - currentNode.getDistanceAsDouble();
    double minSecToGoal;
    
    final double curSpeed = currentNode.getSpeedAsDouble();
    final double deltaSpeed = maxVelocity - currentNode.getSpeedAsDouble();
    final double timeToOperSpeed = deltaSpeed / maxAccel;
    final double distToOperSpeed = curSpeed*timeToOperSpeed + 0.5 * maxAccel*timeToOperSpeed*timeToOperSpeed;
    
    if (distToOperSpeed > distanceToGoal) {
      return Double.POSITIVE_INFINITY; // If we can't reach operating speed before the goal this node is irrelevant. TODO is this true?
      // double speedAtNext = Math.sqrt(curSpeed*curSpeed + 2.0*maxAccel_*distanceToGoal);
      // minSecToGoal = (speedAtNext - curSpeed) / maxAccel_;
    } else {
      //accelerate to operating speed before reaching the goal.
      double cruiseTime = (distanceToGoal - distToOperSpeed) / maxVelocity;
      minSecToGoal = timeToOperSpeed + cruiseTime;
    }
    
    double minJToGoal = getJFromOpMode(1, minSecToGoal); // OpMode of 1 is idle and represents minimum possible fuel usage 
    double minNormJToGoal = minJToGoal / fuelNormalizationDenominator;
    double minNormTimeToGoal = minSecToGoal / timeNormalizationDenominator;

    double minCostToGoal = minNormJToGoal * percentCostForFuel + minNormTimeToGoal * percentCostForTime;
    //System.out.println("normalizedHeuristics: " + normalizedHeuristics);
    return minCostToGoal * heuristicWeight; // Apply heuristic weight. If greater than 1.0 this will be an inadmissable heuristic
    }

    @Override
    public void setTolerances(Node tolerances) {
        this.tolerances = tolerances;
    }

    @Override
    public void setGoal(Node goal) {
        this.goal = goal;
        this.numCosts = 0;
    }

    /**
     * We are at out goal if past the target distance and at operating speed. 
     * Time is ignored as there is no way to predict how long will be spent at a light.
     * Since time is part of the cost model it will still be minimized during the search
     */
    @Override
    public boolean isGoal(Node n) {
        boolean result;

        //if tolerances have been specified then use them
        if (tolerances != null) {
            result = n.getDistance() >= (goal.getDistance() - tolerances.getDistance())  &&
                    Math.abs(n.getSpeed()    - goal.getSpeed())    <= tolerances.getSpeed();
        }else {
            result = n.getDistance() >= goal.getDistance()  &&
                     n.getSpeed()    >= goal.getSpeed();
        }

        if (result) {
            log.debug("EAD", "///// isGoal has found a node that matches our goal.");
            log.debug("EAD", "    goal = " + goal.toString());
            log.debug("EAD", "    node = " + n.toString());
            log.debug("EAD", "    tol  = " + tolerances.toString());
        }

        return result;
    }

    @Override
    public boolean isUnusable(Node n) {
        return n.getDistance() > (goal.getDistance() + tolerances.getDistance()) &&
          Math.abs(n.getSpeed()    - goal.getSpeed())    > tolerances.getSpeed();
    }
}
