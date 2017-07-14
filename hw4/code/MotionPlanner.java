package applications.homeworks.unlocked.hw1;

import java.util.*;
import java.awt.geom.*;

import applications.homeworks.unlocked.hw1.MotionPlanner.AlgorithmType;
import applications.homeworks.unlocked.hw1.MotionPlannerState;

import smrde.math.*;


/**
 * A discrete motion planner. Grassfire, Dykstras, Astar, and Dstar algorithms are supported 
 * in a common infrastructure.
 * 
 * This class is designed for visualization while the planner is running. The method 
 * doIteration will perform node expansions only up to failure or some specified limit 
 * and then return.
 * 
 * For debugging and visualization purposes, doNavFunction can be set to force the planner to 
 * continue expansion until the open list becomes empty. The resulting policy in the closed
 * and open lists then encodes a path from everywhere back to the start.
 * 
 * In addition, the threshold maxExpansions is provided in setQuery(). A "query" is a single
 * attempt to plan a path from some start to some goal. If planning expands 
 * this many accumulated nodes (over the processing of this query), planning is terminated and the 
 * best plan known so far is extracted. This can be used to test real time re-planning algorithms
 * where, by assumption, there is no time to do any more expansions and a partial plan 
 * must be executed.
 * 
 * Take care to distinguish maxExpansions which applies to the entire query from 
 * maxIterationExpansions which applies to a single call to doIteration. 
 * 
 * Grassfire, Astar and Star algorithms are supported. Grassfire treats obstacles as hard constraints 
 * that cannot be crossed. The others treat obstacles as high cost regions and will return the lowest 
 * cost path even if it must drive through obstacles.
 * 
 * Change tracking is supported along with incremental reading of changes. An externally provided 
 * set of changed states is filled up over time as iterations are performed. This set is emptied externally
 * at any desired frequency. External users also decide when it is time to reset the cost information
 * to a new set of baseline values from which changes will be detected.
 * 
 * @author alonzo
 *
 */
public class MotionPlanner{
	/**
	 * Type of algorithm to execute
	 * @author alonzo
	 *
	 */
	public enum AlgorithmType {Grassfire, Dykstra, Astar, Dstar};
	/**
	 * Start state. May be the real goal state when running Dstar.
	 */
	private MotionPlannerState startState;
	/**
	 * Goal state. May be the real start state when running Dstar.
	 */
	private MotionPlannerState goalState;
	/**
	 * Algorithm type to execute.
	 */
	private AlgorithmType algorithmType;
	/**
	 * Enclosing context that created me.
	 */
	public MotionPlannerContext motionPlannerContext;
	/**
	 * Open queue. May be a FIFO or a Priority Queue.
	 */
	private Queue <MotionPlannerState> open;
	/**
	 * Maximum number of expansions permitted to try to solve the planning query.
	 */
	private int maxExpansions;
	/**
	 * Expansions performed so far for this query.
	 */
	private int totExpansions;
	/**
	 * Flag to be set when the path to the goal has been found.
	 */
	private boolean pathReady;
	/**
	 * Flag to be set when the path to the goal has been found for the 
	 * first Dstar pass.
	 */
	private boolean firstDstarPathReady;
	/**
	 * Flag to be set when the open list is empty (indicating) that all reachable
	 * states have been expanded.
	 */
	private boolean navReady;
	/**
	 * Flag to enable continued expansion after the goal is encountered.
	 */
	private boolean doNavFunction;
	/**
	 * Local storage for the extracted path. This is read externally.
	 */
	private ArrayList<MotionPlannerState> path;
	/**
	 * The termination for Dstar can happen too early due to numerical roundoff.
	 * This small threshold is used to cause the search to extend just slightly
	 * "past" the goal to ensure that nodes upstream of the goal that appear to 
	 * be slightly downstream are still processed. 
	 */
	private double DStarTerminationThreshold = 1.0e-9;
	/**
	 * Constructs a motionPlanner.
	 * @param motionPlannerContext parent context that created me
	 * @param maxExpansions max expansion permitted before failing per query
	 * @param algorithmType algorithm to use for solving query
	 * @param doNavFunction flag to control nav function computation
	 */
	public MotionPlanner(MotionPlannerContext motionPlannerContext,int maxExpansions, 
			AlgorithmType algorithmType, boolean doNavFunction){
		this.motionPlannerContext = motionPlannerContext;
		this.doNavFunction = doNavFunction;
		this.algorithmType = algorithmType;
		this.maxExpansions = maxExpansions;
		
		path = new ArrayList<MotionPlannerState>();
		
		if(notDoingGrassfire())
			open = new PriorityQueue <MotionPlannerState> ();
		else
			open = new LinkedList <MotionPlannerState> (); // LinkedList adds/removes in constant time.

	}
	
	/**
	 * Checks to make sure a query is feasible in the sense that the start and goal are
	 * not in collision. It does not really matter whether the start or goal are in "collision" 
	 * for anything other than Grassfire but this routine will return failure anyway to force a
	 * search for a collision free query.
	 * @param start start pose
	 * @param goal  goal pose
	 * @return true if neither is in collision.
	 */
	public boolean checkQuery(Pose2D start, Pose2D goal) {		
		// The goal must be created first in order for heuristics to be correct
		Point2D goalPoint = new Point2D.Double(goal.getX(), goal.getY());
		if(motionPlannerContext.costMap.checkPointForCollision(goalPoint)) {
			System.out.printf("Goal  is in collision. Retrying...\n");
			return false;
		}
		
		// Create the start state
		Point2D startPoint = new Point2D.Double(start.getX(), start.getY());
		if(motionPlannerContext.costMap.checkPointForCollision(startPoint)) {
			System.out.printf("Start  is in collision. Retrying...\n");
			return false;
		}
		return true;
	}
		
	/**
	 * Sets the planning query to be progressively worked on by subsequent calls to doIteration(). 
	 * Initalizes the planner state, clears all lists etc. for a fresh start.
	 * @param start start pose (heading is ignored)
	 * @param goal end pose (heading is ignored)
	 * @return true if either start or goal are in collision. In this case, the user should
	 * vary the start and goal and try again.
	 */
	public boolean setQuery(Pose2D start, Pose2D goal) {				
		// set the states
		Point2D goalPoint = new Point2D.Double(goal.getX(), goal.getY());
		goalState = motionPlannerContext.setGoalState(goalPoint);
		Point2D startPoint = new Point2D.Double(start.getX(), start.getY());
		startState = motionPlannerContext.setStartState(startPoint);
		
		// Clear lists and place initial node on list
		open.clear();
		motionPlannerContext.searchGraph.clearStates();	
		
		// Initialize heuristic bias
		motionPlannerContext.resetK_m();

		// set its cost if necessary and put it on open
		if(notDoingGrassfire()){
			startState.g = 0.0;
		}	

		if(algorithmType == AlgorithmType.Dstar){
			startState.rhs = 0.0;
			startState.setBackPointer(null);
		}
		
		addNodeToOpen(startState, null);
		
		pathReady = false;
		navReady = false;
		this.totExpansions = 0;
		return true;
	}
	/**
	 * Move the goal state and update the heuristic bias. Used for moving the vehicle in Dstar.
	 * @param pt new position 
	 */
	public boolean moveGoal(Point2D pt){
		motionPlannerContext.updateHeuristicForMovedGoal(pt);
		goalState = motionPlannerContext.setGoalState(pt);
		
		if(motionPlannerContext.costMap.checkPointForCollision(pt)) {
			System.out.printf("Robot is on obstacle. Proceeding anyway ...\n");
			return false;
		}
		this.totExpansions = 0;
		return true;
	}
	/*
	 ***************************** Node Expansion (Search) ************************
	 */	
	/**
	 * Perform node expansions until either:
	 * a) the goal is encountered and doNavFunction is off, or
	 * b) maxIterationExpansions have been performed or 
	 * c) total expansions for this query exceeds maxExpansions (provided in setQuery)
	 * d) the open queue is empty meaning either the quest for the goal has failed or
	 * the goal was found but the entire nav function was desired.
	 * 
	 * @param maxIterationExpansions the maximum allowed expansions before returning
	 * @param firstDstarCycle if set, equivalent to doNavFunction being set for this
	 * iteration. Used to compute the intial nav function used for Dstar.
	 */
	public void doIteration(int maxIterationExpansions, boolean firstDstarCycle) {
		int expansions = 0;
		
		// do the expansions. Don't stop until nav function is ready if 
		// this is the first Dstar cycle
		while(expansions <= maxIterationExpansions || firstDstarCycle) {
			// declare failure if necessary
			if(open.isEmpty()){
				if(doingDStar() && firstDstarCycle){
					firstDstarPathReady = true;
				} else if (doingDStar() && !firstDstarCycle) {
					pathReady = firstDstarPathReady;
				}
				if(doNavFunction) 
					System.out.printf("Open is empty\n");
					navReady = true;
				return;
			}
			// get state at front of open
			MotionPlannerState qTopState = open.poll();
			//System.out.printf("Removing node at %s \n", state);
			
			// declare success if goal has been reached
			if(!doingDStar() && qTopState.equals(goalState) && !pathReady){
				System.out.printf("Found Goal\n");
				pathReady = true;
				if(!doNavFunction && !doingDStar())break;
			}
			
			// Terminate Dstar iterations if finished
			if(doingDStar() && !doNavFunction){
				getRhs(goalState);
				boolean done1 = qTopState.getF() > goalState.getF()+DStarTerminationThreshold; // past goal
				boolean done2 = goalState.g < MotionPlannerState.Infinity;
				boolean done3 = goalState.g == goalState.rhs; // goal is optimal
				if(done1 && done2 && done3 && !firstDstarCycle) {
 					System.out.printf("Dstar is done in %d expansions\n",expansions);
 					pathReady = true;
					break;
				}
			}
			
			if(totExpansions == maxExpansions){
				System.out.printf("Outta time\n");
				pathReady = true; // partial solution from start
				break;
			}
			
			// put neighbors on open
			expand(qTopState);
			expansions++;
			totExpansions++;
		}
	}
	
	/**
	 * Expand the provided state. Calls expandGrassfire or expandAstar etc. as 
	 * determined by the algorithm chosen in the constructor of this object.
	 * @param state the state to expand.
	 */
	private void expand(MotionPlannerState state){
		if(algorithmType == AlgorithmType.Grassfire){
			expandGrassfire(state);
		} else if(algorithmType == AlgorithmType.Dykstra){
			expandDykstra(state);
		} else if(algorithmType == AlgorithmType.Astar){
			expandAstar(state);
		} else if(algorithmType == AlgorithmType.Dstar){
			expandDstar(state);
		}
	}
	
	/**
	 * Expands the provided state according to the conventions for the grassfire 
	 * algorithm. 
	 * @param state the state to expand
	 */
	// TODO: Complete expandGrassfire.
	private void expandGrassfire(MotionPlannerState state){		

		// Step 1: move state from open to closed.
		removeNodeFromOpen(state);
		addNodeToClosed(state);
		// Step 2: get its neighbors
		ArrayList <MotionPlannerState> neighbors = motionPlannerContext.searchGraph.generateLegalNeighbors(state);
		// Step 3: add neighbors (which are not in collision)
		// to open in a loop. 
		for(MotionPlannerState neighbor : neighbors){
			// Step 3.1: Do not add nodes that are in collision. Use motionPlannerContext.costMap.isInCollision()
			// to detect collisions.
			if (motionPlannerContext.costMap.isInCollision(new Point2D.Double(neighbor.x(), neighbor.y()))) continue;
			// Step 3.2: Make sure the neighbor node is not already on open or closed before adding it to open.
			if (motionPlannerContext.statesOpened.contains(neighbor) || motionPlannerContext.statesClosed.contains(neighbor)) continue;
			addNodeToOpen(neighbor, state);
		}
	}
	/**
	 * Expands the provided state according to the conventions for Dykstras 
	 * algorithm.
	 * @param state the state to expand
	 */
	// TODO: Complete expandDykstra.
	private void expandDykstra(MotionPlannerState state){
		// Step 1: Move state from open to closed
		removeNodeFromOpen(state);
		addNodeToClosed(state);
		// Step 2: Get its neighbors
		ArrayList <MotionPlannerState> neighbors = motionPlannerContext.searchGraph.generateLegalNeighbors(state);
		// Step 3: Add neighbors to open in a loop. 
		for(MotionPlannerState neighbor : neighbors){
			// Step 3.1 Compute the G for this neighbor if state
			// were its parent. Use deriveStartCost().
			double G = motionPlannerContext.deriveStartCost(neighbor, state);
			// Step 3.2 If neighbor is already on open and state is a cheaper 
			// parent, update its cost and parent by removing it, changing its g
			// value and putting it back on open. Use removeNodeFromOpen() and
			// addNodeToOpenWithNewGValue().
			if(neighbor.open){
				if (G < neighbor.g) {
					removeNodeFromOpen(neighbor);
					addNodeToOpenWithNewGValue(neighbor, G, state);
				}
			// Step 3.3 Do the same thing if neighbor is on closed.
			} else if(neighbor.closed){
				if (G < neighbor.g) {
					removeNodeFromOpen(neighbor);
					addNodeToOpenWithNewGValue(neighbor, G, state);
				}
			// Step 3.3 Otherwise, neighbor has no parent yet, so put it on open.
				
			} else {
				addNodeToOpenWithNewGValue(neighbor, G, state);
			}
		}
	}
	/**
	 * Expands the provided state according to the conventions for the Astar 
	 * algorithm.
	 * @param state the state to expand
	 */
	// TODO: Complete expandAstar.
	private void expandAstar(MotionPlannerState state){
		// Step 1: Move state from open to closed
		removeNodeFromOpen(state);
		addNodeToClosed(state);
		// Step 2: Get its neighbors
		ArrayList <MotionPlannerState> neighbors = motionPlannerContext.searchGraph.generateLegalNeighbors(state);
		// Step 3: Add neighbors to open in a loop. 
		for(MotionPlannerState neighbor : neighbors){
			// Step 3.1 Compute the G, H and F for this neighbor if state
			// were its parent
			double newG = motionPlannerContext.deriveStartCost(neighbor, state);
			double newH = neighbor.getH();
			double newF = newG + newH;
			// Step 3.2 If neighbor is already on open and state is a cheaper 
			// parent, update its cost and parent by removing it, changing its g
			// value and putting it back on open. Use removeNodeFromOpen() and
			// addNodeToOpenWithNewGValue().
			if(neighbor.open){
				if (newF < neighbor.getF()) {
					removeNodeFromOpen(neighbor);
					addNodeToOpenWithNewGValue(neighbor, newG, state);
				}
			// Step 3.3 Do the same thing if neighbor is on closed.
			} else if(neighbor.closed){
				if (newF < neighbor.getF()) {
					removeNodeFromOpen(neighbor);
					addNodeToOpenWithNewGValue(neighbor, newG, state);
				}
			// Step 3.3 Otherwise, neighbor has no parent yet, so put it on open.
			} else {
				addNodeToOpenWithNewGValue(neighbor, newG, state);
			}
		}
	}
	
	/**
	 * Expands the provided state according to the conventions for the Dstar 
	 * algorithm.
	 * @param state the state to expand
	 */
	// TODO: Complete expandDstar.
	private void expandDstar(MotionPlannerState state){
		// Step 1: Remove state from open and compute its rhs value
		removeNodeFromOpen(state); // probably already removed
		double kOld = state.getF();
		getRhs(state);
		// Step 2: If state had an underconsistent key, put it back 
		// on open.
		if(kOld < state.getF()){
		}
		// Step 2a: If cost g can be reduced to rhs, reduce it and 
		// update all neighbors of state
		if(state.g > state.rhs){
		// Step 2b: Otherwise set g to infinity (MotionPlannerState.Infinity)
		// and a) update all neighbors and b) update this state
		} else {  // state is optimal or cost must go up
		}			
	}

	/**
	 * Compute the lowest cost possible based on every historical parent
	 * node. This is an experimental function. Do not use. Use getRhs.
	 * @param state the node whose lowest cost is desired.
	 */
	public void getRhsParents(MotionPlannerState state){
		if(state == startState) return; // start state has fixed rhs of zero
		double minGValue = MotionPlannerState.Infinity;
		MotionPlannerState bestParent = state.getBackPointer();
		LinkedHashSet <MotionPlannerState> neighbors;
		neighbors = state.getParents();

		for(MotionPlannerState neighbor : neighbors){
			double newG = motionPlannerContext.deriveStartCost(state, neighbor);		
			if(newG < minGValue) {
				minGValue = newG;
				bestParent = neighbor;
			}
		}
		state.rhs = minGValue;
		state.setBackPointer(bestParent);
	}
	
	/**
	 * Compute the lowest cost possible based on every possible neighbor
	 * node.
	 * @param state the node whose lowest cost is desired.
	 */
	// TODO: Complete getRhs.
	public void getRhs(MotionPlannerState state){
		// Step 1: do nothing if this is  the start state
		if(state == startState) return;
		// Step 2. Check all neighbors of state and
		// a) set bestParent to the neighbor whose start cost is lowest
		// b) set state.rhs to the start cost of the best neighbor
		// use motionPlannerContext.deriveStartCost
		double minGValue = MotionPlannerState.Infinity;
		MotionPlannerState bestParent = state.getBackPointer();
		ArrayList <MotionPlannerState> neighbors;
		neighbors = motionPlannerContext.searchGraph.generateLegalNeighbors(state);

		for(MotionPlannerState neighbor : neighbors){
			// fill me in
		}
		state.rhs = minGValue;
		state.setBackPointer(bestParent);
	}
	
	/*
	 ************************** Dstar Cost Change Propagation *********************
	 */	
	/**
	 * Propagate cost changes from parents to this node. This node is possibly 
	 * in the wrong place in the open list since its cost was changed. If so, put
	 * it in the right place by removing and re-adding it.
	 * @param state the node to be updated.
	 */
	// TODO: Complete updateVertex.
	public void updateVertex(MotionPlannerState state){
		// Step 1: compute rhs if this is not the start state
		if(state != startState )
			// Fill me in
		// Step 2: if node (state) is on open, remove it
		if(state.open) 
			// Fill me in
		// Step 3: if node (state) is not yet optimal, place it back
		// on open. 
		if(state.g != state.rhs){  // node is not yet optimal
			// Fill me in
		}
	}
	
	/**
	 * Update the search tree to reflect changes in the map.
	 */
	public void updateDstarSearchTreeForMapChanges(){
		LinkedHashSet <MotionPlannerState> changedStates;
		changedStates = motionPlannerContext.getStatesWithModifiedCells();
		
		if(doingDStar()){
			for(MotionPlannerState changedState: changedStates){
				updateVertex(changedState);
			}
		}
	}
	
	/**
	 * Update the search tree to reflect newly percieved costs.
	 */
	public void updateDstarSearchTreeForPerceivedCosts(){
		LinkedHashSet <MotionPlannerState> changedStates;
		changedStates = motionPlannerContext.getStatesWithSeenCells();
		
		if(doingDStar()){
			for(MotionPlannerState changedState: changedStates){
				updateVertex(changedState);
			}
		}
		int numChanges = changedStates.size();
		if(numChanges != 0) pathReady = false;
		//System.out.printf("%d Vertices updated for perception\n", numChanges);
	}
	/*
	 ******************************* Open Queue and Closed Set ***************************
	 */	
	/**
	 * Used to add nodes to the open queue. 
	 * @param state the state to add
	 * @param g its new g value
	 * @param parent the state to which it should point. This is the parent state on the way
	 * back to the start which is associated with the least cost path.
	 */
	public void addNodeToOpenWithNewGValue(MotionPlannerState state, double g, MotionPlannerState parent){
		state.g = g;
		addNodeToOpen(state,parent);
	}
	
	/**
	 * Used to add nodes to the open queue. 
	 */
	public void addNodeToOpenWithPresentParent(MotionPlannerState state){
		addNodeToOpen(state,state.getBackPointer());
	}
	
	/**
	 * Used to add nodes to the open queue. The node must have been removed from the queue if
	 * it was in it before. Moving a node due to cost changes is accomplished by removal and
	 * re-addition.
	 * @param state the state to add
	 * @param parent the state to which it should point. This is the parent state on the way
	 * back to the start which is associated with the least cost path.
	 */
	private void addNodeToOpen(MotionPlannerState state, MotionPlannerState parent){
		state.setBackPointer(parent);
		if(state.hasChanged()) motionPlannerContext.statesChanged.add(state);
		state.closed = false;
		state.open = true;
		open.add(state);
		motionPlannerContext.statesOpened.add(state);
	}
	
	/**
	 * Used to removed nodes from the open queue.
	 */
	private void removeNodeFromOpen(MotionPlannerState state){
		if(state.hasChanged()) motionPlannerContext.statesChanged.add(state);
		open.remove(state);
		state.open = false;
	}
	
	/**
	 * Used to add nodes to the closed set. 
	 */
	private void addNodeToClosed(MotionPlannerState state){
		if(state.hasChanged()) motionPlannerContext.statesChanged.add(state);
		if(!state.closed)motionPlannerContext.statesClosed.add(state);
		state.closed = true;
		state.open = false;

	}
	
	/**
	 * Used to removed nodes from the closed set. 
	 */
	private void removeNodeFromClosed(MotionPlannerState state){
		state.closed = false;
		if(state.hasChanged())motionPlannerContext.statesChanged.add(state);
	}
	
	/*
	 ********************************* Path Services ********************************
	 */	
	/**
	 * Extracts the path from the backpointers working backwards from the
	 * goal (or the best leaf in depth limited search) up to AND INCLUDING the start. 
	 * Puts the result in an ArrayList<PlannerState>. Adds at the front in order to 
	 * reverse the path and order it from start to finish. Terminates when the start 
	 * state is added to the path or returns an empty path if planning failed. In the 
	 * case where the start and goal state are identical, the path will contain only 
	 * one occurence of that state.
	 * @return the solution path.
	 */
	// TODO: Complete extractPath.
	public ArrayList<MotionPlannerState> extractPath(){
		// Step 1: Clear last path, if any
		path.clear();
		// Step 2: Get last  state in path
		MotionPlannerState state = null;
		state = goalState;
		/**
		 * Step 3: Add nodes at the front until done.
		 * If the best horizon node is null, planning failed so return an empty list.
		 * If the best horizon node is the state state, the plan is trivial, so return
		 * a plan with just the start state in it. Otherwise, walk back from the best
		 * horizon node to (and including) the start.
		 */
		while(state != null){
			// Fill me in here.
			path.add(state);
			state = state.getBackPointer(); // may return null;
			if (state == null) {
				path.clear();
				return path;
			}
			if (state == startState) {
				path.add(state);
				Collections.reverse(path);
				return path;
			}
			// If Dstar is stopped before it completes, it may leave a partial path
			// which is in the middle of reversing itself. Such a path will be an infinite
			// cycle. Just break the path here if so and return.
			if(state != null && path.contains(state)) {break;}
		}
		
		// Reverse result for Dstar since it plans from actual goal to actual start by
		// flipping them in the Query specfication.
		if(doingDStar()){Collections.reverse(path);}
		
		return path;
	}
	
	/**
	 * Finds a node on the open list in order to determine what move to make next. 
	 * If the goal state has a backpointer then planning succeeded all the way to the
	 * goal and the goal is the best horizon node. If the open list is empty, it means 
	 * planning has failed and a null pointer will be returned.
	 */
	// TODO: Complete getBestHorizonNode.
	private MotionPlannerState getBestHorizonNode() {
		
		MotionPlannerState state;
		MotionPlannerState best = null;
		double bestValue = Double.MAX_VALUE;
		
		/**
		 * If goal state has a backpointer it was reached, so it is
		 * the best horizon node.
		 */
		if(goalState.getBackPointer() != null) return goalState;	
		/**
		 * If the start and goal are the same, planning succeeds trivially and
		 * the goal is the best horizon node.
		 */
		if(goalState == startState) return goalState;
		/**
		 * Otherwise, the goal was not reached and there may or may not be some
		 * nodes on open. If there are, return the best node. If not, return null.
		 */
	    for (Iterator<MotionPlannerState> openIt = open.iterator(); openIt.hasNext(); ) {
	    	state = (MotionPlannerState) openIt.next();
	    	if (!open.isEmpty()) {
	    		double distance = state.getF();
	    		if (distance < bestValue) {
	    			bestValue = distance;
	    			best = state;
	    		}
	    	} else {
	    		return null;
	    	}
	    }
		
		return best;
	}	
	
	/*
	 ************************************** Planner Status ************************************
	 */
	/**
	 * @return true when the plan is ready. May be ready because the goal was reached or
	 * because maxExpansions has been exceeded.
	 */
	public boolean pathReady() { return pathReady; };
	/**
	 * @return true when the nav function is ready
	 */
	public boolean navReady()  { return navReady; };
	/**
	 * @return true when planning has failed. That means open is empty and the 
	 * path has not found. Usually this means the goal is simply not connected 
	 * to the start by any path.
	 */
	public boolean planningFailed() { return open.isEmpty()  && !pathReady; };
	/**
	 * @return true if algorithm is not Grassfire. 
	 */
	public boolean notDoingGrassfire() { 
		return algorithmType == AlgorithmType.Dykstra 
		    || algorithmType == AlgorithmType.Astar 
	    	|| algorithmType == AlgorithmType.Dstar; 
	};
	/**
	 * @return true if running the Dstar algorithm
	 */
	public boolean doingDStar() { 
		return algorithmType == AlgorithmType.Dstar; 
	};
	/**
	 * @return a reference to the internally stored path
	 */
	public ArrayList<MotionPlannerState> getPath(){ 
		return path;
	};
}

