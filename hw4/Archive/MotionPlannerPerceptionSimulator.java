package applications.homeworks.unlocked.hw1;

import java.lang.annotation.*; // import this to use @Documented
import java.awt.geom.*;
import java.util.*;

import smrde.math.*;

/**
 * A class to simulate a perception system for a moving mobile robot to be used for
 * testing a motion planner. 
 * @author alonzo
 *
 */
public class MotionPlannerPerceptionSimulator {
	/**
	 * The cost map
	 */
	MotionPlannerCostMap costMap;
	/**
	 * Reference to pose of sensor in World
	 */
	Pose2D sensorPose;
	/**
	 * Maximum distance the sensor can see
	 */
	double sensorRadius;
	/**
	 * Constructs a MotionPlannerPerceptionSimulator
	 * @param costMap the cost map to be revealed as the sensor moves
	 * @param sensorPose reference to pose of sensor in World.
	 * @param sensorRadius maximum distance the sensor can see
	 */
	public MotionPlannerPerceptionSimulator(MotionPlannerCostMap costMap, Pose2D sensorPose, double sensorRadius){
		this.costMap = costMap;
		this.sensorPose = sensorPose;
		this.sensorRadius = sensorRadius;
	}
	/**
	 * Simulate perception by looking in a square region of size 2*sensorRadius by 2*sensorRadius
	 * and reading the cost cvalues there.
	 *
	 */
	public void simulatePerception(){
		// Simulate perception
		Point2D minPt = new Point2D.Double();
		Point2D maxPt = new Point2D.Double();
		
		minPt.setLocation(sensorPose.getX()-sensorRadius, sensorPose.getY()-sensorRadius);
		maxPt.setLocation(sensorPose.getX()+sensorRadius, sensorPose.getY()+sensorRadius);
		
		minPt = costMap.costArray.nearestCellCenterInWorld(minPt);
		maxPt = costMap.costArray.nearestCellCenterInWorld(maxPt);
		
		Rectangle2D rect = Util.RectFromCorners(minPt,maxPt); 
		
		// Get the list of cells in this rectangle
		ArrayList<Point2D> cellList = costMap.costArray.getCellsInUserRectangle(rect);
		
		// look at every point to make the cells "seen".
		for(Point2D pt: cellList){
			costMap.getCellCost(pt);
		}
	}

}
