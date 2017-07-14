package applications.homeworks.unlocked.hw1;

import java.lang.annotation.*; // import this to use @Documented
import java.awt.geom.Point2D;

/**
 * An interface to allow computation of the cost heuristic external to the planner
 * graph. The intention is to keep the planner graph unaware of any planner specific
 * data.
 * @author alonzo
 *
 */
public interface MotionPlannerHeuristicCalculator {
	/**
	 * Computes the value of the cost field h with respect to the goal state.
	 * @return the value of the heuristic function h().
	 */
	public double hValue(double x, double y);
	/**
	/**
	 * Computes the heuristic value from the new goal to the 
	 * old and adds this amount to the heuristic bias.
	 * @param pt position of new goal
	 */
	public void updateHeuristicForMovedGoal(Point2D pt);
	/**
	 * Clears the value of the heuristic bias to zero. Used when resetting all memory
	 * of costs in Dstar.
	 */
	public void resetK_m();
}
