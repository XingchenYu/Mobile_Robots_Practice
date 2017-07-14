package applications.homeworks.unlocked.hw0;

/**
 * A class that processes a ladar scans to look for either corners or reflections. The 
 * former makes sense for large landmarks (with enough range hits to make corner detection 
 * possible. The latter makes sense for small landmarks where any hit at all can be regarded 
 * as a reflection. Landmarks are assumed to contain right angles.
 */
import java.awt.geom.Point2D;

import smrde.math.Angle;
import smrde.math.PolarPose2D;
import smrde.math.PolarVector2D;
import smrde.objects.RangeImage;

public class KalmanLadarFeatureFinder {
	
	public enum FeatureType {Reflection,Corner};
	
	FeatureType type;
	
	/**
	 * Number of pixels to jump for fitting lines used to detect corners.
	 */
	final int lineLength = 3;
	/**
	 * Threshold for deviation from 90 degrees and still be deemed a "corner".
	 */
	final double threshAngle = Math.PI/10.0;

	/**
	 * Constructs a feature finder o fhte indicated type
	 * @param type
	 */
 	public KalmanLadarFeatureFinder(FeatureType type) {
		this.type = type;
	}
 	
	/**
 	 * Returns the pose of any detected feature in the SENSOR frame.
 	 * @param rangeImage the range image
 	 * @param maxRange maximum range of ladar sensor
 	 * @param sigmaRR range sensor variance
 	 * @return the polar pose of any detected feature or null
 	 */
 	public PolarPose2D findFeaturesInLadar(RangeImage rangeImage, double maxRange, double sigmaRR){
 		
 		if(type == FeatureType.Reflection){
 			return findReflectionFeaturesInLadar(rangeImage,maxRange,sigmaRR);
 		}
 		else if(type == FeatureType.Corner){
 			return findCornerFeaturesInLadar(rangeImage,maxRange,sigmaRR);
 		}
 		return null;
 	}
 	
	/**
 	 * Returns the pose of any detected feature in the SENSOR POLAR frame.
 	 * @param rangeImage the range image
 	 * @param maxRange maximum range of ladar sensor
 	 * @param sigmaRR range sensor variance
 	 * @return the polar pose of any detected feature or null
 	 */
 	public PolarPose2D findReflectionFeaturesInLadar(RangeImage rangeImage, double maxRange, double sigmaRR){
 		
 		// TODO: Fill me in
 		return null;
 	}
 	
	/**
 	 * Returns the pose of any detected corner feature in the SENSOR frame.
 	 * @param rangeImage the range image
 	 * @param maxRange maximum range of ladar sensor
 	 * @param sigmaRR range sensor variance
 	 * @return the polar pose of any detected feature or null
 	 */
 	public PolarPose2D findCornerFeaturesInLadar(RangeImage rangeImage, double maxRange, double sigmaRR){
 		
 		int numRanges = rangeImage.ranges.length;
 		
 		for(int i=lineLength ; i<numRanges-lineLength; i++) {

			// compute angle of line between present pixels

			int ibwd = (i-lineLength);
			int inow = (i+  0       );
			int ifwd = (i+lineLength);
					
			Point2D ptBwd = new Point2D.Double();
			Point2D ptNow = new Point2D.Double();
			Point2D ptFwd = new Point2D.Double();
			rangeImage.imageToSensor(ibwd, rangeImage.ranges[ibwd], ptBwd);
			rangeImage.imageToSensor(inow, rangeImage.ranges[inow], ptNow);
			rangeImage.imageToSensor(ifwd, rangeImage.ranges[ifwd], ptFwd);
			double xbwd = ptBwd.getX(); double ybwd = ptBwd.getY();
			double xnow = ptNow.getX(); double ynow = ptNow.getY();
			double xfwd = ptFwd.getX(); double yfwd = ptFwd.getY();		
		  
			double lenbwd = Math.sqrt((xbwd-xnow)*(xbwd-xnow)+(ybwd-ynow)*(ybwd-ynow));
			double lenfwd = Math.sqrt((xfwd-xnow)*(xfwd-xnow)+(yfwd-ynow)*(yfwd-ynow));
		
			double thbwd = Math.atan2(ybwd-ynow,xbwd-xnow);
		  	double thfwd = Math.atan2(yfwd-ynow,xfwd-xnow);
		  	
		  	// compute subtended angle and compare to goal
	    	double dth = Math.abs(Angle.normalize(thbwd-thfwd));
	    	//System.out.println("Dth " + dth);
	    	double goalAngle1 = Math.PI/2.0;
	    	double goalAngle2 = 3.0*Math.PI/2.0;
	
			boolean closeToGoal = Math.abs(dth-goalAngle1)<threshAngle || Math.abs(dth-goalAngle2)<threshAngle;
			
			boolean linesAreShort =  lenbwd < (3.5 * lineLength * rangeImage.ranges[i] * rangeImage.getAngRes())
						  		  && lenfwd < (3.5 * lineLength * rangeImage.ranges[i] * rangeImage.getAngRes());
			
			//System.out.printf("thb%f thf%f dth %f Clos: %b short:%b\n",thbwd , thfwd ,dth, closeToGoal,linesAreShort);
	
	    	if(closeToGoal  && linesAreShort && rangeImage.ranges[i] < 0.95*maxRange) {
				//draw something at xnow,ynow
				//System.out.println("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Got one");
				
				// Compute angle of feature in sensor frame. At this point, the angle
				// must be a convex or concave right angle. If thfwd - thbwd is
				// < PI  then use thbwd, else use thfwd
				
				//double fmb = Math.abs(Angle.normalize(thfwd-thbwd));
				double th = thfwd;
				
				//if(fmb < Math.PI) th = thfwd; else th = thfwd;
								
				// compute position of feature in polar coordinates frame
				
				PolarVector2D pt = new PolarVector2D();
				rangeImage.imageToPolar(i, rangeImage.ranges[i], pt);	
				return new PolarPose2D(pt.r,pt.th,th);
	      	}
		 		
		 }
 		return null;
 	}
}
