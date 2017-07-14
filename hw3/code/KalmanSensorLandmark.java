package applications.homeworks.unlocked.hw0;

import java.awt.Color;
import java.util.ArrayList;

import applications.homeworks.unlocked.hw0.KalmanLadarPerceptor;

import smrde.math.*;
import smrde.objects.RangeImage;
import smrde.systems.RigidBodySprite;
import smrde.systems.Sprite;
import smrde.systems.World;

/**
 * The interface between a landmark sensor and a Kalman filter. Provided so that the filter code 
 * is independent of the sensors used. Computes the innovation, the measurement Jacobian and the 
 * measurement covariance. This sensor is much more complicated that the other two, but the 
 * complexities are placed in classes external to this one.
 * 
 * This class can form bearing residuals or range residuals or both.
 * 
 * For detailed, slow speed debugging, the positions of the predicted feature and the measured feature
 * can be rendered by updating their positions and their appearance characteristics here.
 *
 */
public class KalmanSensorLandmark {
	/**
	 * State vector reference.
	 */
	private Matrix state;
	/**
	 * Controls debug printouts
	 */
	private boolean doDebug;
	/**
	 * Compute bearing innovations if on
	 */
	private boolean doBearing;
	/**
	 * Compute range innovations if on.
	 */
	private boolean doRange;
	/**
	 * Landmarks are moveable (SLAM) if on.
	 */
	private boolean doSLAM;
	/**
	 * Range sensor variance
	 */
	private double sigmaRR;
	/**
	 * Index of x in state vector
	 */
	int xStateIndex;
	/**
	 * Index of y in state vector
	 */
	int yStateIndex;
	/**
	 * Index of theta in state vector
	 */
	int thetaStateIndex;
	/**
	 * Length of state vector.
	 */
	private int numStates;
	/**
	 * Pose of sensor on body. Also, homogeneous transform from sensor coords to body coords.
	 */
	private Pose2D sensorToBody;
	/**
	 * Perception system for finding features.
	 */
	private KalmanLadarPerceptor kalmanLadarPerceptor;
	/**
	 * Length of measurement vector.
	 */
	private int numMeasurements;
	/**
	 * Index of bearing innovation in measurement vector
	 */
	private int bearingIndex;
	/**
	 * Index of range innovatioon in measurement vector.
	 */
	private int rangeIndex;
	/**
	 * Draw feature glyphs if on.
	 */
	private boolean doFeatureGraphics = false;
	/**
	 * Glyph for measured feature.
	 */
	private Sprite  measuredFeatureSprite;
	/**
	 * Glyph for predicted feature.
	 */
	private Sprite  predictedFeatureSprite;
	/**
	 * Constructs a KalmanSensorLandmark.
	 * @param state reference to state vector
	 * @param doDebug controls debug printouts
	 * @param doBearing controls generation of bearing innovations
	 * @param doRange controls generation of range innovations
	 * @param doSLAM adds landmarks to state vector if on (SLAM)
	 * @param doCornerFeatures detect corners rather than reflections
	 * @param sigmaRR range sensor variance
	 * @param xStateIndex index of x state
	 * @param yStateIndex index of y state
	 * @param thetaStateIndex index of theta state
	 * @param maxRange maximum sensor range
	 * @param numStates length of state vector
	 * @param sensorToBody sensor pose on body
	 * @param landmarks list of landmarks
	 * @param displayList external graphics display list
	 */
	public KalmanSensorLandmark(Matrix state, boolean doDebug, boolean doBearing, boolean doRange, 
			boolean doSLAM, boolean doCornerFeatures,
			double sigmaRR, int xStateIndex, int yStateIndex, int thetaStateIndex,
			double maxRange, int numStates, Pose2D sensorToBody,
			ArrayList<RigidBodySprite> landmarks, World displayList){
		this.state = state;
		this.doDebug = doDebug;
		this.doBearing = doBearing;
		this.doRange = doRange;
		this.doSLAM = doSLAM;
	 	this.sigmaRR = sigmaRR;
	 	this.xStateIndex = xStateIndex;
	 	this.yStateIndex = yStateIndex;
	 	this.thetaStateIndex = thetaStateIndex;
	 	this.numStates = numStates;
	 	this.sensorToBody = sensorToBody;
	 	this.kalmanLadarPerceptor = new KalmanLadarPerceptor(state,doCornerFeatures, sensorToBody,maxRange,landmarks,sigmaRR);
	 	
	 	// To use feature orientation, add a doOrientation flag and cover all the cases.
	 	if(doBearing && doRange){
	 		numMeasurements = 2;
	 		bearingIndex = 0;
	 		rangeIndex = 1;
	 	}
	 	else if(doBearing){
	 		numMeasurements = 1;
	 		bearingIndex = 0;
	 	}
	 	else if(doRange){
	 		numMeasurements = 1;
	 		rangeIndex = 0;
	 	}
	 	
		if(doFeatureGraphics){
			measuredFeatureSprite  = Sprite.CircleSprite("LeftFront", 0.4, 0.0, 0.0, 0.0);
			predictedFeatureSprite = Sprite.CircleSprite("RghtFront", 0.4, 0.0, 0.0, 0.0);
			measuredFeatureSprite.drawingContext().setFillColor(Color.blue);
			predictedFeatureSprite.drawingContext().setFillColor(Color.red);
			measuredFeatureSprite.drawingContext().setDoDrawBorder(false);
			predictedFeatureSprite.drawingContext().setDoDrawBorder(false);
			displayList.add(measuredFeatureSprite);
			displayList.add(predictedFeatureSprite);
		}
	}
	
	/**
	 * Compute the innovation.
	 * @param rangeImage the range image with potential landmarks in it
	 * @return the innovation vector z - h(x)
	 */
	public Matrix getInnovation(RangeImage rangeImage){

		if(doFeatureGraphics){
			measuredFeatureSprite.drawingContext().setDoDrawBorder(true);
			predictedFeatureSprite.drawingContext().setDoDrawBorder(true);
			measuredFeatureSprite.drawingContext().setDoDrawFill(false);
			predictedFeatureSprite.drawingContext().setDoDrawFill(false);
		}
		
		if( !kalmanLadarPerceptor.findFeatures(rangeImage) ) return null;
		
		if(doFeatureGraphics){
			Pose2D measuredPose = new Pose2D(kalmanLadarPerceptor.measuredFeaturePoseInWorld);
			Pose2D predictedPose = new Pose2D(kalmanLadarPerceptor.predictedFeaturePoseInWorld);
			double dx = measuredPose.getX()-predictedPose.getX();
			double dy = measuredPose.getY()-predictedPose.getY();
			measuredPose.add(10.0*dx,10.0*dy,0.0);
			measuredFeatureSprite.poseContext().setPoseWorld(measuredPose);
			predictedFeatureSprite.poseContext().setPoseWorld(predictedPose);
			measuredFeatureSprite.drawingContext().setFillColor(Color.blue);
			predictedFeatureSprite.drawingContext().setFillColor(Color.red);
		}
		
		//TODO: fill me in
		Matrix dz= new Matrix(numMeasurements, 1);
		PolarPose2D mea_pose = kalmanLadarPerceptor.measuredFeaturePoseInPolar;
		PolarPose2D pri_pose = kalmanLadarPerceptor.predictedFeaturePoseInPolar;
		if(doBearing && doRange){
	 		dz.setElement(0, 0, mea_pose.th - pri_pose.th);
	 		dz.setElement(1, 0, mea_pose.r - pri_pose.r);
	 	}
	 	else if(doBearing){
	 		dz.setElement(0, 0, mea_pose.th - pri_pose.th);
	 	}
	 	else if(doRange){
	 		dz.setElement(0, 0, mea_pose.r - pri_pose.r);
	 	}
   		return dz;
	}
	/**
	 * Compute the measurement Jacobian
	 * @return the measurement Jacobian
	 */
	public Matrix getMeasurementJacobian(){
  		// compute measurement Jacobian
		Matrix H = new Matrix(numMeasurements,numStates);
		
		//TODO: fill me in
		Pose2D mea_pose = kalmanLadarPerceptor.measuredFeaturePoseInSensor;
		double th = Math.atan2(mea_pose.getY(), mea_pose.getX());
		double r = Math.sqrt(mea_pose.getY() * mea_pose.getY() + mea_pose.getX() * mea_pose.getX());
		Matrix H1 = new Matrix(2,2);
		H1.setElement(0, 0, -Math.sin(th)/r);
		H1.setElement(0, 1, Math.cos(th)/r);
		H1.setElement(1, 0, Math.cos(th));
		H1.setElement(0, 1, Math.sin(th));
		Matrix H2 = new Matrix(2,2);
		Matrix H3 = new Matrix(2,3);
		
		H.print(" H ",doDebug);
 		return H;
	}
	/**
	 * Compute the measurement covariance.
	 * @param rangeImage the range image with potential landmarks in it
	 * @param dt elapsed time since last call to this routine.
	 * @return the measurement covariance.
	 */
	public Matrix getMeasurementCovariance(RangeImage rangeImage,double dt){
  		// compute measurement uncertainty
  		Matrix R = new Matrix(numMeasurements,numMeasurements);
  		//TODO: fill me in
  		
  		if(doBearing && doRange){
	 		R.setElement(0, 0, 0.01);
	 		R.setElement(1, 1, sigmaRR);
	 	}
	 	else if(doBearing){
	 		R.setElement(0, 0, 0.01);
	 	}
	 	else if(doRange){
	 		R.setElement(0, 0, sigmaRR);
	 	}
  		R.print(" R ",doDebug);
  		return R;
	}
	
 	/**
 	 * Get the index in the landmark list of the landmark that was matched to the last feature. 
 	 * @return the index of the last matched landmark
 	 */
 	public int matchedLandmark(){
 		return kalmanLadarPerceptor.matchedLandmarkIndex();
 	}
}
