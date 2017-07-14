package applications.homeworks.unlocked.hw1;

import java.util.*;
import java.awt.geom.*;

import applications.homeworks.unlocked.hw1.MotionPlanner.AlgorithmType;

import smrde.math.*;
import smrde.systems.*;


/**
 * A contextual object used to organize motion planning. This class ensures that cost 
 * map, search graph, and obstacle list are all ignorant of each other. It also implements
 * many of the design options for the various planners so that the planner state class does
 * not need to know the algorithm that is using it. This class has the primary purposes of:
 * 
 * a) Creating the motion planner, the cost map and the search graph.
 * b) Creating the start and goal states for a motion planning query.
 * c) Modifying the cost map when the user has added or deleted obstacles or when
 * perception has seen more of the environment.
 * d) Computing edge costs by integrating over the cost map while following 
 * an edge in the search graph.
 * e) Computing the heuristic and the start costs of states.
 * f) Maintaining lists of states which have experienced different events like 
 * creation, moving from open to closed, or cost change. These are used for graphics 
 * purposes externally.
 * 
 * Since it is more general than alternatives, path costs are computed as the line 
 * integral of a scalar cost field along the path. Although discrete obstacles may 
 * be provided here, they are processed into the cost map and thereafter forgotten,
 * so the planner itself never sees the polygonal obstacle objects that created the cost
 * map.
 * 
 * Both Euclidean and Manhattan distance metrics are available for the heuristic (estimate of
 * cost to goal) and cost from start.
 * 
 * @author alonzo
 *
 */
public class MotionPlannerContext implements MotionPlannerHeuristicCalculator{
	
	static final long serialVersionUID = 1L;
	/**
	 * The motion planner algorithm.
	 */
	public MotionPlanner motionPlanner;
	/**
	 * The cost map containing an array of cost cells.
	 */
	public MotionPlannerCostMap costMap;
	/**
	 * The seach graph object. This is elaborated as needed rather than computed beforehand.
	 */
	public MotionPlannerSearchGraph searchGraph;
	/**
	 * Switch to control the use of heuristics. Heuristic is zero if this is off.
	 */
	private boolean doHeuristic;
	/**
	 * Switch to control the use of Euclidean or Manhattan metrics in the heuristic
	 * and cost to come calculations. Default is true.
	 */
	private boolean doManhattanMetric;
	/**
	 * State to be used for calculating heuristics.
	 */
	private MotionPlannerState goalState;
	/**
	 * State to be used as start of plan.
	 */
	private MotionPlannerState startState;
	
	/**
	 * A factor to multiply h() by before adding g() to produce f(). Higher values 
	 * increase the significance of h relative to g. Default is 1.0.
	 */
	private double  greedyFactor; // >1.0 is more greedy
	
	/**
	 * The fraction of a free Cell cost to associate with heading changes in units of PI/4. This
	 * is used as a tie breaker so that navigation fields will look nice on screen. This should
	 * be a very small number or it will cause the heuristic to to be a slight overestimate. You 
	 * could also multiply the heuristic result in cells by (1+this number) to potentially fix
	 * that issue. Then it will no longer be an overestimate.
	 */
	private double turnGradient = 1.0e-14;
	
	/**
	 * The heuristic bias - called "key modifier".
	 */
	private double k_m;
	
	/**
	 * List of all objects created, closed, etc.
	 */
	public LinkedHashSet<MotionPlannerState> statesChanged = new LinkedHashSet<MotionPlannerState>();
	public LinkedHashSet<MotionPlannerState> statesTransient = new LinkedHashSet<MotionPlannerState>();
	public LinkedHashSet<MotionPlannerState> statesOpened  = new LinkedHashSet<MotionPlannerState>();
	public LinkedHashSet<MotionPlannerState> statesClosed  = new LinkedHashSet<MotionPlannerState>();
	public LinkedHashSet<MotionPlannerState> statesMarked  = new LinkedHashSet<MotionPlannerState>();
	

	/**
	 * Create a MotionPlannerContext.
	 * @param body a body in the shape of the vehicle whose motion is being planned.
	 * @param sensorPose sensorPose the pose of the sensor in the world. Used for limtied perception horizon.
	 * @param sensorRadius the maximum distance that the sensor can see
	 * @param worldList list of obstacles to use at start
	 * @param worldRect rectangular region in which to plan
	 * @param obsCost the cost of an obstacle cell
	 * @param resolution spatial resolution of the cost map and search graph
	 * @param maxExpansions maximum number of node expansions per query
	 * @param algorithmType planning algorithm to use
	 * @param doNavFunction flag to control computation of entire navigation function
	 * @param do8ConnectedTopology flag to control 4 or 8 connected topology of search graph
	 * @param doManhattanMetric flag to control use of Manhattan or Euclidean distance metric
	 * @param greedyFactor factor to control search greediness. > 1 is greedy
	 */
	public MotionPlannerContext(Forceable body, Pose2D sensorPose, double sensorRadius,
			World worldList, Rectangle2D worldRect, double obsCost, double resolution,
			int maxExpansions, AlgorithmType algorithmType, boolean doNavFunction, boolean do8ConnectedTopology, 
			boolean doManhattanMetric, double greedyFactor){
		this.doManhattanMetric  = doManhattanMetric;
		this.greedyFactor = greedyFactor;
		
		doHeuristic = true;
		if(algorithmType == AlgorithmType.Dykstra){
			doHeuristic = false;
		}
		
		// Set up the motion planner
		motionPlanner = new MotionPlanner(this,maxExpansions,algorithmType,doNavFunction);
		
		// Set up the cost map. For now, it has the same resolution as the states
		// but it could be anything as long as edge costs are computed correctly.
		// If this changes getModified states and the gValue funtion at least will
		// need to be updated.
		costMap = new MotionPlannerCostMap(body,sensorPose, sensorRadius, worldList,worldRect,obsCost, resolution);
		
		// Set up the search graph. For now, a rectangular array is used which
		// may be 4 or 8 connected.
		searchGraph = new MotionPlannerSearchGraph(worldRect,resolution, do8ConnectedTopology, this);
	}
	
	/**
	 * Sets the goal state to be used in the heuristic calculation.
	 * @param goalPoint point designating position of goal
	 * @return the goal state.
	 */
	public MotionPlannerState setGoalState(Point2D goalPoint){
		goalState = searchGraph.getState(goalPoint);
		return goalState;
	}
	
	/**
	 * Sets the start state to be used in motion planning.
	 * @param startPoint point designating position of start
	 * @return the start state.
	 */
	public MotionPlannerState setStartState(Point2D startPoint){
		startState = searchGraph.getState(startPoint);
		return startState;
	}
	
	/**
	 * @return the start state.
	 */
	public MotionPlannerState getStartState(){
		return startState;
	}
	
	/**
	 * @return the goal state.
	 */
	public MotionPlannerState getGoalState(){
		return goalState;
	}
	
	/**
	 * Get a set of those states that have been affected by obstacles that have been 
	 * added to or deleted from the world.
	 * @return the set of affected states
	 */
	public LinkedHashSet <MotionPlannerState> getStatesWithModifiedCells(){
		MotionPlannerState state;
		
		for(Point2D pt: costMap.cellsModified){
			state = searchGraph.getState(pt);
			statesChanged.add(state);
		}
		return statesChanged;
	}
	
	/**
	 * Get a set of those states affected by cells that have been just been seen by 
	 * perception. In a more realistic case, you would check for any cells in the field 
	 * of view whose costs have changed but costs only change here when something becomes 
	 * visible for the first time.
	 * @return the set of affected states
	 */
	public LinkedHashSet <MotionPlannerState> getStatesWithSeenCells(){
		MotionPlannerState state;
		
		for(Point2D pt: costMap.cellsSeen){
			state = searchGraph.getState(pt);
			statesChanged.add(state);
		}		
		return statesChanged;
	}
	/*
	 *********************************** Cost for Two States *********************************
	 */	
	/**
	 * Compute the start relative cost (g) of the provided state through the indicated
	 * parent.  
	 * @param state the state
	 * @param parent the parent to use
	 * @return the g value to the start through the parent
	 */
	public double deriveStartCost(MotionPlannerState state, MotionPlannerState parent){
		/*
		 * Compute g value of state if "parent" is the parent.
		 */
		double newgValue = gValue(state,parent);
		/*
		 * Make turning cost something tiny in order to have smoother solutions.
		 */
		double turnCost  = turnGradient*costMap.freeCost()/(Math.PI/4.0);
		newgValue += angleChange(parent,state)*turnCost;
		
		return newgValue;
	}
	
	/**
	 * Computes the angle change between two successive backpointers: the backpointer
	 * of state and that of its parent.
	 * @param parent the potential parent state
	 * @param state the potential child state
	 * @return the angle change in radians. Returns zero if any needed pointers are null.
	 */
	public double angleChange(MotionPlannerState parent, MotionPlannerState state){
		if (parent == null) return 0.0;
		if (parent.getBackPointer() == null) return 0.0;
		MotionPlannerState save = state.getBackPointer();
		state.setBackPointerOnly(parent);
		double thParent = parent.getBackPointerOrientation();
		double thState  = state.getBackPointerOrientation();
		state.setBackPointerOnly(save);
		return(Angle.normalize(thState-thParent));
	}
	
	/*
	 **************************** Internal Planner Cost Calculations ***************************
	 */	
	
	/**
	 * Computes the value of the start cost with respect to the indicated parent. This
	 * is the g value of the parent plus the cost of the edge connecting them.
	 * @param state the state whsoe g is desired use
	 * @param parent the parent state to base the increment to g on.
	 * @return the value of the heuristic function g().
	 */
	//TODO: complete gValue().
	private double gValue(MotionPlannerState state, MotionPlannerState parent){
		// Step 1: Compute the center distance according to the prevailing distance metric.
		double dx = parent.x()-state.x();
		double dy = parent.y()-state.y();
		double dg = distanceMetric(dx,dy);
		// Step 2: Get the costs of each of the cells.
		Point2D parentPt = new Point2D.Double(parent.x(),parent.y());
		Point2D statePt  = new Point2D.Double(state.x(),state.y());
		double cost1 = costMap.getCellCost(parentPt);
		double cost2 = costMap.getCellCost(statePt);
		// Step 3: Compute the cost of the edge from the center distance and the two cell costs
		
		// Your code goes here. One line will do.
		dg = dg * (cost1 + cost2);
		return(parent.g+dg);
	}
	
	/**
	 * Computes the distance between two points based on the prevailing definition of distance.
	 * If doManhattanMetric is on, uses Manhattan distance. Otherwise uses Euclidean distance.
	 * @param dx x displacement
	 * @param dy y displacement
	 * @return
	 */
	//TODO: complete distanceMetric().
	private double distanceMetric(double dx, double dy){
		double distance;
		dx = Math.abs(dx);
		dy = Math.abs(dy);
		if(doManhattanMetric){
			distance = dx + dy; // fill me in
		}else {
			distance = Math.sqrt(dx * dx + dy * dy); // fill me in
		}
		return distance;
	}
	/**
	 * Computes the distance to be used for the heuristic for the provided displacement.
	 * @param dx the x displacement
	 * @param dy the y displacement
	 * @return the value of the heuristic function h().
	 */
	private double hDistanceValue(double dx, double dy){	
		if(!doHeuristic){
			return 0.0;
		}else{
			return(distanceMetric(dx,dy));
		}
	}
	/*
	 *********************** Interface MotionPlannerHeuristicCalculator *********************
	 */	
	/**
	 * Computes the value of the cost field h with respect to the indicated goal state.
	 * @param x the goal state x coord to base h on.
	 * @param y the goal state x coord to base h on.
	 * @return the value of the heuristic function h().
	 */
	//TODO: complete hValue().
	public double hValue(double x, double y){
		// Step 1: Compute the distance from the goal state
		double dx = goalState.x()-x;
		double dy = goalState.y()-y;
		double h = hDistanceValue(dx,dy);
		
		// Step 2: Mutiply by the cost of a free cell. This means the same
		// thing as assuming there are no obstacles when computing the heuristic.
		h = h * costMap.freeCost(); // fill me in
		
		// Step 3: Scale by greedy factor and bias by the key modifier.
		return(greedyFactor*(h+k_m));
	}
	
	/**
	 * Clears the value of the heuristic bias to zero. Used when resetting all memory
	 * of costs in Dstar.
	 */
	public void resetK_m(){
		k_m = 0.0;
	}
	/**
	 * Computes the heuristic value from the new goal to the 
	 * old and adds this amount to the heuristic bias.
	 * @param pt position of new goal
	 */
	//TODO: complete updateHeuristicForMovedGoal().
	public void updateHeuristicForMovedGoal(Point2D pt){
		// Step 1: Compute the distance moved by the robot (i.e. the distance from 
		// the present goalState to the argument pt
		double dx = goalState.x()-pt.getX();
		double dy = goalState.y()-pt.getY();
		double dk_m = hDistanceValue(dx, dy);
		// Step 2: Multiply this distance by the cost of a free cell.
		// Step 3: Add this increment to the key modifier k_m
	}
}