package applications.homeworks.unlocked.hw0;
/**
 * A class to associate landmarks with perception sensor readings. The closest match is returned
 * without regard to whether it is far away or not. Reasonableness checks should be performed on the 
 * results produced by this class.
 * 
 * This routine presently 
 * cheats a little in bearing only mode since it uses the range data to locate the measured 
 * feature in 2D.
 */
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

import smrde.math.Pose2D;
import smrde.systems.RigidBodySprite;

public class KalmanLandmarkAssociator {
	
	/**
	 * List of landmarks to process
	 */
	private ArrayList<RigidBodySprite> landmarks;
	/**
	 * Index of closest feature in last match.
	 */
	private int closestFeatureIndex;	

	/**
	 * Constructs a KalmanDataAssociator
	 * @param landmarks a list of landmarks in the "map"
	 */
	public KalmanLandmarkAssociator(ArrayList<RigidBodySprite> landmarks) {
		this.landmarks = landmarks;
	}

 	/**
 	 * Get the index in the landmark list of the landmark that was matched to 
 	 * the last feature that was processed. 
 	 * @return the index of the last matched landmark
 	 */
 	public int matchedLandmarkIndex(){
 		return closestFeatureIndex;
 	}
 	
	/**
	 * Finds the closest feature in the map to the specified point. This routine presently 
	 * cheats a little in bearing only mode since it uses the range data to locate the measured 
	 * feature in 2D. As a side effect, this routine sets the value of closestFeatureIndex so that
	 * a Kalman filter can compute an innovation.
	 * @param featurePoseWorld specified pose.
	 * @param sensorPoseWorld pose of sensor in the WORLD frame
	 * @return the pose of the closest feature
	 */
 	public Pose2D closestCornerPose(Pose2D featurePoseWorld, Pose2D sensorPoseWorld){
 		Pose2D bestPose = new Pose2D();
 		
 		// Step 1: Determine the index of the landmark whose center is closest to
 		// detected feature's position. Update closestFeatureIndex for
 		// users external to this class
 		
 		// TODO: Fill me in
 		
 		// Step 2: Determine closest corner of the landmark to the SENSOR
 		// (i.e. not closest predicted corner to the measured corner).
 		// and return that as the pose of the closest landmark corner.
 		
 		// TODO: Fill me in
 		return bestPose;
 	}
}
