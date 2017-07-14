package applications.homeworks.unlocked.hw1;

import java.util.*;
import java.lang.annotation.*; // import this to use @Documented
import smrde.math.*;


/**
 * A state to be used in motion planning. Records the position and cost
 * data. Also records a backpointer (parent) to be used in extracting the path.
 * 
 * The x and y coordinates are discretized equally. All states which round to the same
 * discrete coordinates are considered equal.
 * 
 * Cost fields f, g, and h and a backpointer field are available for use in 
 * implementing heuristic search algorithms.
 * 
 * "Last" values of cost and backpointer are maintained to facilitate tracking which 
 * states are changed during replanning. A cell is changed if either its cost or its 
 * backpointer changes - or both. Last values of cost are remembered across even the 
 * clearing of all cell contents. This makes it possible to run a planner twice (once
 * with an obstacle and once without) and detect exactly which cells changed as a result
 * of the obstacle. 
 * 
 * A set of children (opposite of backpointers) is maintained to assist in debugging. A
 * state may have only one parent, but many children are possible.
 * 
 * A set of parents is maintained that is managed externally. It typically contains all
 * states that have appeared on open which have expanded at some point to this state. Only
 * one member of this list can be the instantaneous value of the backpointer.
 * 
 * Heuristic values "h" are computed externally.
 * 
 * @author alonzo
 *
 */
public class MotionPlannerState implements Comparable<MotionPlannerState>{
	/**
	 * The cost threshold to declare a state ha changed
	 */
	static double ChangeThreshold = 1.0e-6;
	/**
	 * The value INFINITY for Dstar
	 */
	public static double Infinity = 1.0e16;
	/**
	 * Discretized x coordinate.
	 */
	final private double x;
	/**
	 * Discretized y coordinate.
	 */
	final private double y;
	/**
	 * Integer equivalent of x coordinate. Used for fast comparisons.
	 */
	final public int i;
	/**
	 * Integer equivalent of y coordinate. Used for fast comparisons.
	 */
	final public int j;
	/**
	 * Cost to come.
	 */
	public double g;
	/**
	 * Best g value possible right now.
	 */
	public double rhs;
	/**
	 * Calculator of heuristic for cost to go.
	 */
	MotionPlannerHeuristicCalculator heurCalc;
	/**
	 * The last value of f. Used to signal change propagations in replanning 
	 * operations to external visualization.
	 */
	public double lastF;
	/**
	 * Backpointer state on the path to the start.
	 */
	private MotionPlannerState parent;
	/**
	 * Last value of backpointer.
	 */
	private MotionPlannerState lastParent;
	/**
	 * All current children
	 */
	private LinkedHashSet <MotionPlannerState> children;
	/**
	 * All parents that have ever been seen
	 */
	private LinkedHashSet <MotionPlannerState> parents;
	/**
	 * Closed flag (set this when a node is placed on closed list).
	 */
	public boolean closed;
	/**
	 * Open flag (set this when a node is placed on open list).
	 */
	public boolean open;
	/**
	 * A field for general use to store an application specific data structure. One
	 * such use could be a point to a graphical object for the state.
	 */
	public Object userData;
	
	/**
	 * Constructs a plannerState
	 * @param i the discrete y coordinate used for comparisons
	 * @param j the discrete x coordinate used for comparisons
	 * @param x the x coordinate
	 * @param y the y coordinate
	 * @param heurCalc the heuristic value calculator
	 */
	public MotionPlannerState(int i, int j, double x, double y, MotionPlannerHeuristicCalculator heurCalc){
		this.i = i;
		this.j = j;
		this.x = x;
		this.y = y;
		this.heurCalc = heurCalc;
		children = new LinkedHashSet <MotionPlannerState>();
		parents  = new LinkedHashSet <MotionPlannerState>();
		clear();
		lastF = Infinity; // used to make sure creation event does not look like a cost change
		}
	/**
	 * Clears the contents of this state. Clears all of the following:
	 * - children and parent lists
	 * - open and closed flags
	 * - parent and lastParent pointers
	 * - sets rhs and g to Infinity
	 * LastF is NOT cleared here in order to detect cost changes across two motion planner
	 * queries.
	 */
	public void clear(){
		children = new LinkedHashSet <MotionPlannerState>();
		parents = new LinkedHashSet <MotionPlannerState>();
		rhs = Infinity;
		g = Infinity;
		closed = false;
		open = false;
		parent = null;
		lastParent = null;
	}
	
	/*
	 ********************************* Gets n Sets **************************************
	 */	
	/**
	 * Gets the sum GMin() +  h().
	 */
	public double getF(){
		return getMinG_Rhs() + getH();
	}
	/**
	 * Gets the g value for AStar or Grassfire algorithms or min(g,rhs) for Dstar.
	 */
	public double getMinG_Rhs(){
		return Math.min(g,rhs);
	}
	/**
	 * Gets the heuristic value.
	 */
	public double getH(){
		return heurCalc.hValue(x, y);
	}
	/**
	 * Gets the discretized x coordinate.
	 * @return the discretized x coordinate.
	 */
	public double x(){ return x;}
	
	/**
	 * Gets the discretized y coordinate.
	 * @return the discretized y coordinate.
	 */
	public double y(){ return y;}
	
	/**
	 * Gets the pose of a state. For 2D states, sets orientation to zero.
	 * @return the pose (x,y,0) of the state.
	 */
	public Pose2D Pose(){ 
		Pose2D pose = new Pose2D(x(),y(),0.0);
		return pose;}

	/**
	 * Sets the backpointer of a state. Also sets the child pointer list 
	 * of the parent to point back to this.
	 * @param parent
	 */
	public void setBackPointer(MotionPlannerState parent){
		// already my parent. do nothing
		if(this.parent == parent) return;
		// I have a different parent. Disown it.
		if(this.parent != null){
			this.parent.children.remove(this);
		}
		// New, first or null parent
		this.parent = parent;
		this.parents.add(parent);
		parent.children.add(this);
	}
	
	/**
	 * Sets the backpointer of a state. Does not set the child pointer 
	 * of the parent.
	 * @param parent
	 */
	public void setBackPointerOnly(MotionPlannerState parent){
		this.parent = parent;
	}
	/**
	 * Gets the children of a state.
	 */
	public LinkedHashSet <MotionPlannerState> getChildren(){
		return this.children;
	}
	/**
	 * Gets the parents of a state.
	 */
	public LinkedHashSet <MotionPlannerState> getParents(){
		return this.parents;
	}
	/**
	 * Adds to the parents of a state
	 * @param state the parent to add
	 */
	public void addParent(MotionPlannerState state){
		this.parents.add(state);
	}
	
	/**
	 * Gets the backpointer of a state.
	 */
	public MotionPlannerState getBackPointer(){
		return this.parent;
	}
	
	/**
	 * Gets the direction in space of the backpointer of a state. Used
	 * to permit drawing backpointers on a display.
	 */
	public double getBackPointerOrientation(){
		double x1,y1,x2,y2;
		x1 = x; y1 = y;
		x2 = parent.x();
		y2 = parent.y();
		return Math.atan2(y2-y1, x2-x1);
	}
	
	/**
	 * Return a list containing all of a states descendents (defined as those states reachable
	 * by traversing backpointers in reverse.
	 * @return list containing all of a states descendents
	 */
	public ArrayList <MotionPlannerState> getDescendents(){
		ArrayList <MotionPlannerState> list = new ArrayList <MotionPlannerState>();
		
		// Do this in two steps to control the order for display purposes.
		// Immediate children first, then their children etc.
		for (MotionPlannerState state : children){
			list.add(state);
		}
		
		for (MotionPlannerState state : children){
			list.addAll(state.getDescendents());
		}
		
		return list;
	}
	
	/*
	 ********************************* Change Detection **************************************
	 */	
	
	/**
	 * Saves the cost info for detecting changes. Calls to MotionPlanner.setQuery 
	 * set g and hence f to infinity. In order to detect changes across calls to setQuery,
	 * an infinite value of f is not saved so that the last noninfinite value is saved 
	 * instead.
	 */
	public void saveLastCostInfo(){
		double F = getF();
		if(F > 0.9*Infinity) return;
		if(checkIfMonitoredState()){
			System.out.printf("Saving state lastF:%f F: %f\n",lastF,F);
		}
		lastF = F;
		lastParent = parent;
	}
	
	/**
	 * Detect if a state has changed since the last time its cost was recorded. A
	 * state is not considered to have changed if:
	 * a) its cost is being set for the first time
	 * b) its new value is Infinity meansign it was just cleared
	 */
	public boolean hasChanged(){
		double newF = getF();
		if(checkIfMonitoredState()){
			System.out.printf("State being checked lastF:%f F: %f\n",lastF,getF());
		}
		if(lastF  > 0.9*Infinity) return false; // this is only true first time through
		if(newF   > 0.9*Infinity) return false; // cell was just cleared
		boolean valueChanged =  Math.abs(lastF-newF)> ChangeThreshold;
		boolean pointerChanged = lastParent != null && lastParent != parent;
		boolean changed = valueChanged || pointerChanged;
		
		// Set a breakpoint here if you like
		if(changed && checkIfMonitoredState()){
			System.out.printf("State changed lastF:%f F: %f\n",lastF,getF());
		}
		return changed;
	}
	/**
	 * Check to see if this state is a particular one being monitored. Use this for
	 * debugging by putting a breakpoint in this routine.
	 * @return true if this is the monitored state
	 */
	public boolean checkIfMonitoredState(){
		boolean turnedOn = false;
		double dx = x - 0.25;
		double dy = y - 0.25;
		double ds = Math.hypot(dx,dy);
		if(!turnedOn) return false;
		boolean thisIsIt = ds < 0.5;
		if(thisIsIt)
			thisIsIt = true; // for breakpoint
		return thisIsIt;
	}
	
	/*
	 ********************************* Stay Outta Here!! **************************************
	 */	
	/**
	 * Compares two plannerStates for equality. This method overrides the equals()
	 * method (associated with every Java object) for this class. The function of many
	 * Java collections classes is influenced by the behavior of this method. Specifically,
	 * the remove method of ArrayList will use this to find the PlannerState to remove.
	 * @return true if the states are equal.
	 */
	public boolean equals(Object other){
		boolean result;
		
		if(other instanceof MotionPlannerState){
			MotionPlannerState state = (MotionPlannerState)other;
			result = (this.i==state.i) && (this.j==state.j);
		}
		else
			result = false;
		
		return result;
	}
	
	/**
	 * Computes an integer which encodes the values in this instance. Used in conjunction with
	 * equals by Java Collections classes. Java assumes that two objects whose hashCodes are 
	 * unequal cannot be equal, so this is a fast way to establish inequality. 
	 */
	public int hashCode(){
		return((int)(this.i*100000000+this.j*10000));
	}
	
	/**
	 * Compares two plannerStates for the purpose of ordering. This method is used by 
	 * Java's sort routines. 
	 * @param state the state to compare to this
	 * @return -1 if this < state; +1 if this > state; 0 otherwise.
	 */
	public int compareTo(MotionPlannerState state){
		double thisF = this.getF();
		double stateF = state.getF();
		
		if(thisF < stateF) return -1;
		if(thisF > stateF) return +1;
		
		//now we know this.f == state.f. Now base decision on g.
		double thisG = this.getMinG_Rhs();
		double stateG = state.getMinG_Rhs();
		
		if(thisG < stateG) return -1;
		if(thisG > stateG) return +1;
		
		return 0;
	}
	
	/**
	 * Create a string representation of this instance. Convenient generally but especially 
	 * useful for the debugger.
	 */
	//public String toString() {return new String("PlannerState @ x:" + x + " " + " y:" + y + " i:" + i + " " + " j:" + j + " f:" + getF() + " g:" + g + " rhs:" + rhs  + " " + " h:" + h);};
	
	public String toString() {
		String stat;
		if(open) stat = "open";
		else if(closed) stat = "closed";
		else stat = "none";
		
		String thStr = "none";
		if(parent != null)String.format(thStr,getBackPointerOrientation());
		
		String str = String.format("PlannerState @ stat: %s x:%6.2f y:%6.2f i:%3d j:%3d f:%12.8f g:%6.2f rhs:%6.2f h:%6.2f th:%s",
				stat,x,y,i,j,getF(),g,rhs,getH(),thStr);
		return str;
	}

}
