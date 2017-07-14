package applications.homeworks.unlocked.hw1;

import java.lang.annotation.*; // import this to use @Documented
import java.util.*;
import java.awt.*;
import java.awt.geom.*;

import applications.homeworks.unlocked.hw1.MotionPlannerHeuristicCalculator;
import applications.homeworks.unlocked.hw1.MotionPlannerState;

import smrde.math.*;

/**
 * A graph of discrete states for use in motion planning. This class has the primary 
 * purposes of:
 * 
 * a) Mapping continuous position coordinates onto discrete states.
 * b) Using some topology rule to establish which states are connected to others by edges.
 * c) Enforcing the constraint that there may be only one instance of a state 
 * created for any given location. This mechanism avoids many bugs that could occur if
 * two copies existed.
 * d) Maintaining a spatial index into planner states while not requiring that all 
 * states be pre-allocated.
 * 
 * Both 4 connected and 8 connected topologies are available
 * 
 * For now, this class uses a RectangleMap because the state space is 2D, but
 * this would have to be changed if a higher dimensional state space were used.
 * 
 * @author alonzo
 *
 */
@SuppressWarnings("unused")
public class MotionPlannerSearchGraph{
	
	static final long serialVersionUID = 1L;
	/**
	 * Number of rows in the state array.
	 */
	public final int rows;
	/**
	 * Number of columns in the state array.
	 */
	public final int cols;
	/**
	 * An array of pointers to the allocated planner states.
	 */
	private MotionPlannerState stateArray[][];
	/**
	 * A mapping from user coordinates to discrete coordinates.
	 */
	private RectangleMap  rectMap;
	/**
	 * Flag to control whether a state has 4 or 8 neighbors.
	 */
	private boolean do8ConnectedTopology;
	/**
	 * Object to compute the graph heuristic
	 */
	private MotionPlannerHeuristicCalculator heuristicCalculator;
	/**
	 * Constructs a MotionPlannerSearchGraph.
	 * @param worldRect rectangle in the world to discretize
	 * @param resolution spatial resolution of states
	 * @param do8ConnectedTopology flag to control topology. 8 connected if on. 4 if off.
	 * @param heuristicCalculator external object able to compute a motion planning heuristic
	 * value from the state position. 
	 */
	public MotionPlannerSearchGraph(Rectangle2D worldRect, double resolution, 
			boolean do8ConnectedTopology,MotionPlannerHeuristicCalculator heuristicCalculator){
		
		/**
		 * The mapping from x,y to i,j is x onto i and y onto j. This convention is created by the
		 * call to super and the calculation of rows and cols below.
		 */		
		rectMap = new RectangleMap(worldRect,new Rectangle2D.Double(0.0,0.0, 
				worldRect.getWidth()/resolution, worldRect.getHeight()/resolution));

		/**
		 * The use of Math.ceil is the next two lines is critical to making sure that there
		 * are enough discrete cells in the state array - but not an extra one if there are already 
		 * enough.
		 */
		rows = (int) Math.ceil(rectMap.getDelu());
		cols = (int) Math.ceil(rectMap.getDelv());
		
		stateArray = new MotionPlannerState[rows][cols];	
		
		this.do8ConnectedTopology = do8ConnectedTopology;
		this.heuristicCalculator = heuristicCalculator;

	}
	
	/**
	 * Gets the plannerState for the indicated location. Creates it if
	 * it does not yet exist.
	 * @param worldPoint the location desired
	 * @return the desired state or null if it is out of bounds
	 */
	public MotionPlannerState getState(Point2D worldPoint){
		
		MotionPlannerState state = peekState(worldPoint);
		
		if(state == null){
			state = makeState(worldPoint);
		}

		return state;
	}
	
	/**
	 * Return the state reference for the present position. It may be null
	 * if no state was created there yet. An alternative to getState without its
	 * side effect of creating missing states.
	 * 
	 * @param worldPoint to position of the state
	 * @return the state
	 */
	public MotionPlannerState peekState(Point2D worldPoint){
		Point viewPoint = rectMap.inRectToOutRectFloor(worldPoint);
		
		// check to make sure its not out of array bounds
		if (!rectMap.isInOutRect(viewPoint)){
			System.out.printf("Attempt to read a state outside worlRect\n");
			return null;
		}
		
		int i = viewPoint.x;
		int j = viewPoint.y;
		
		return stateArray[i][j];
	}

	/**
	 * Create a state at the indicated position.
	 * @param worldPoint the position of the state
	 * @return the newly created state or null on error
	 */
	public MotionPlannerState makeState(Point2D worldPoint){
		Point viewPoint = rectMap.inRectToOutRectFloor(worldPoint);
		// check to make sure its not out of array bounds
		if (!rectMap.isInOutRect(viewPoint)){
			System.out.printf("Attempt to make a state outside worlRect\n");
			return null;
		}

		Point2D statePoint = new Point2D.Double(worldPoint.getX(),worldPoint.getY());
		trimToNearestDiscreteStatePoint(statePoint);
		double discX = statePoint.getX();
		double discY = statePoint.getY();
		
		int i = viewPoint.x;
		int j = viewPoint.y;
		MotionPlannerState state = new MotionPlannerState(i,j,discX,discY,heuristicCalculator);
		stateArray[i][j] = state;
		//System.out.printf("State created at %s\n",state);
		return state;
	}
	
	/**
	 * Clears the contents of all cells by calling MotionPlannerState.clear()
	 * Called at the start of every motion planning query.
	 */
	public void clearStates(){
		for(int i=0 ; i<rows ; i++)
			for(int j=0 ; j<cols ; j++)
				if(stateArray[i][j] != null) stateArray[i][j].clear();
	}
	
	/**
	 * Records the present costs of all cells as a baseline with respect to which
	 * future changes can be detected.
	 */
	public void recordCostsForChangeDetection(){
		for(int i=0 ; i<rows ; i++)
			for(int j=0 ; j<cols ; j++)
				if(stateArray[i][j] != null) stateArray[i][j].saveLastCostInfo();
	}

	/*
	 ********************************* Topology **************************************
	 */	
	/**
	 * Generates the neighbors of a given state based on whatever rules of connectedness
	 * prevail. Either 4 or 8 neighbors are generated based on the value of the switch
	 * do4Connected. Neighbors which are outside the cost map are not generated.
	 * @param state the state whose neighbors is desired
	 * @return an array of PlannerStates where each is a neighbor.
	 */
	public ArrayList <MotionPlannerState> generateLegalNeighbors(MotionPlannerState state) {
		
		// Step 1: Create an empty ArrayList <MotionPlannerState>
		ArrayList <MotionPlannerState> neighList = new ArrayList <MotionPlannerState>();
		
		int[] ii;
		int[] jj;
		
		// Step 2a: For 8 connected case, create two arrays of length 8
		// of x & y integer cell displacements to neighbors. Replace
		// the 4 connected arrays with these.
		if(do8ConnectedTopology){ 
			int x_8 = state.i;
			int[] ii8 = {x_8 - 1, x_8, x_8 + 1, x_8 - 1, x_8 + 1, x_8 - 1, x_8, x_8 + 1}; // TODO: Fix me
			int y_8 = state.j;
			int[] jj8 = {y_8 - 1, y_8 - 1, y_8 - 1, y_8, y_8, y_8 + 1, y_8 + 1, y_8 + 1}; // TODO: Fix me
			ii = ii8;
			jj = jj8;
		// Step 2b: For 4 connected case, create two arrays of length 4
		// of x & y integer cell displacements to neighbors
		} else{
			int x_4 = state.i;
			int[] ii4 = {x_4, x_4 - 1, x_4 + 1, x_4}; // TODO: Fix me
			int y_4 = state.j;
			int[] jj4 = {y_4 - 1, y_4, y_4, y_4 + 1}; // TODO:Fix me
			ii = ii4;
			jj = jj4;
		}
		
		// Step 3: Create all these neighbors and add them to the list
		// of neighbors provided they pass the legalState() test.
		Point2D worldPoint;	
		for(int k=0 ; k < ii.length ; k++){
			int i = ii[k]; // Fix me
			int j = jj[k]; // Fix me
			worldPoint = cellCenterInWorld(i,j);
			if(!legalState(worldPoint)) continue;
			MotionPlannerState newState = getState(worldPoint);
			if(newState != null) neighList.add(newState);
		}
				
		return neighList;
	}
	
	/**
	 * Determines if the supplied state is within the rectangle specified at start time.
	 * @param worldPoint the state to check.
	 * @return true if the state is inside the rectangle
	 */
	public boolean legalState(Point2D worldPoint){
		if(!rectMap.isInInRect(worldPoint)) return false;
		return true;
	}
	
	/*
	 ********************************* Discretization **************************************
	 */	
	/**
	 * Determines the location in world coordinates of the precise center of a cell.
	 * @param i discrete x coordinate (row in array)
	 * @param j discrete y coordinate (col in array)
	 * @return a point positioned exactly at the cell center in word coordinates
	 */
	// TODO: Complete this function
	public Point2D cellCenterInWorld(int i, int j){
		// Step 1: create a point in view (outRect) coordinates at the CENTER of cell i,j
		// Step 2: map it inRect coordinates and send it out
		double x = i + 0.5;
		double y = j + 0.5;
		Point2D view = new Point2D.Double(x, y);
		Point2D world = rectMap.outRectToInRect(view);
		return new Point2D.Double(world.getX(), world.getY()); // return worldPoint
	}
	/**
	 * Moves a point to the nearest discrete state.
	 * @param worldPoint a point in the continuum
	 */
	public void trimToNearestDiscreteStatePoint(Point2D worldPoint){
		Point2D discWorldPoint = rectMap.nearestCellCenterInWorld(worldPoint);
		worldPoint.setLocation(discWorldPoint);
	}
	
	/**
	 * Moves the point part of a pose to the nearest discrete state.
	 * @param worldPose a pose in the continuum
	 */
	public void trimToNearestDiscreteStatePose(Pose2D worldPose){
		Point2D discWorldPoint = new Point2D.Double(worldPose.getX(),worldPose.getY());
		discWorldPoint = rectMap.nearestCellCenterInWorld(discWorldPoint);
		worldPose.setPose(discWorldPoint.getX(),discWorldPoint.getY(),worldPose.getTh());
	}
}