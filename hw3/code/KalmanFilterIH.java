package applications.homeworks.unlocked.hw0;

import java.util.ArrayList;

import smrde.math.*;
import smrde.systems.RigidBodySprite;

/**
 * The top level integrated heading Kalman Filter function. Knows the structure of the state vector. 
 * Literally implements the classical system model and measurement processing equations of the Kalman filter. 
 * There are five state variables (x,y,theta,V,omega). The
 * positions of these variables in the state vector is hardocded herein but is unknown outside
 * this file. External routines may ask things like "where is velocity" in the state vector. 
 * It is harcoded here in the angleFlags static variable that the third state is an angle.
 * 
 * Many objects will hold a reference to the state vector and its covariance in order to track 
 * changes, so these objects are "final" here to prevent external users from holding references
 * to earlier copies which are no longer being updated.
 * 
 * Owner of the state vector and its covariance.
 *
 */
public class KalmanFilterIH {
	/**
	 * State vector
	 */
	final Matrix state;
	/**
	 * State vector covariance.
	 */
	final Matrix stateCovariance;
 	/**
 	 * Controls debug printout for system model. 
 	 */
 	private boolean doSystemModelDebug;
 	/**
 	 * Do fast version if on.
 	 */
 	private boolean doFast;
 	/**
 	 * Do SLAM if on.
 	 */
 	private boolean doSLAM;
 	/**
 	 * Print out the top left 2X2 of the innovation covariance if on.
 	 */
	boolean doPrintS=false;
	/**
	 * Initial state variance in x
	 */
	private final double P0xx = 0.01;
	/**
	 * Initial state variance in y
	 */
	private final double P0yy = 0.01;
	/**
	 * Initial state variance in heading
	 */
	private final double P0tt = 0.01;
	/**
	 * Initial state variance in speed
	 */
	private final double P0vv = 0.01;
	/**
	 * Initial state variance in angular velocity
	 */
	private final double P0bb;
 	/**
 	 * Coefficient of growth rate of P  matrix in x.
 	 */
	private double Qxx = 0.0001;
 	/**
 	 * Coefficient of growth rate of P  matrix in y.
 	 */
	private double Qyy = 0.0001;
 	/**
 	 * Coefficient of growth rate of P  matrix in heading.
 	 */
	private double Qtt = 0.0001;
 	/**
 	 * Coefficient of growth rate of P  matrix in speed.
 	 */
	private double Qvv = 0.05;
 	/**
 	 * Coefficient of growth rate of P  matrix in angular velocity.
 	 */
	private double Qbb = 0.05;
	/**
	 * Position of state X in state vector
	 */
	public final int XX = 0; // position x
	/**
	 * Position of state Y in state vector
	 */
	public final int YY = 1; // position y
	/**
	 * Position of state T (heading) in state vector
	 */
	public final int TT = 2; // orientation theta
	/**
	 * Position of state V (speed) in state vector
	 */
	public final int VV = 3; // velocity
	/**
	 * Position of state B (angular velocity) in state vector
	 */
	public final int BB = 4; // angular velocity
	/**
	 * Number of vehicle (non landmark) states
	 */
	public final int SS = 5; // number of (nonlandmark) states
	/**
	 * Number of landmark STATES (2 per landmark)
	 */
	public final int LL;     // number of (landmark) STATES (2 per landmark)
	/**
	 * Total number of states. Length of state vector.
	 */
	public final int NN; 	 // number of total states
	/**
	 * Number of landmarks.
	 */
	public final int numLandmarks; 
	/**
	 * Angle flags for states. The state is an angle (and should be normalized after all arithmetic)
	 * if set.
	 */
	static boolean angleFlags[] = {false,false,true,false,false};
	
	private Matrix delState;
	private Matrix sysVariances;
	private Matrix Q_veh;
	private Matrix Gam_veh;
	private Matrix GamQGamT_veh;
	private Matrix GamQGamT;
	private Matrix Phi_veh;
	private Matrix stateCovariance_veh;
	private Matrix PPPT_veh;
	private Matrix Pvl;

	/**
	 * Constructs an integrated heading Kalman filter.
	 * @param sigmaGG gyro uncertainty rate 
	 * @param doSystemModelDebug debug printout flag
	 * @param doFast do fast version if on.
	 * @param doSLAM turns on SLAM
	 * @param numLandmarks number of landmarks
	 */
	public KalmanFilterIH(double sigmaGG, boolean doSystemModelDebug, boolean doSLAM, boolean doFast, int numLandmarks){
	 	this.P0bb = sigmaGG;
	 	this.doFast = doFast;
	 	this.doSLAM = doSLAM;
	 	this.doSLAM = doSLAM;
	 	
	 	if(doSLAM){
		 	this.numLandmarks = numLandmarks;
		 	this.LL = 2*this.numLandmarks;
	 	} else{
		 	this.numLandmarks = 0;
		 	this.LL = 0;
	 	}
	 	
	 	NN = SS + LL;
		state = new Matrix(NN,1);
		stateCovariance = new Matrix(NN,NN);
		
		delState = new Matrix(NN,1);
		sysVariances = new Matrix(SS,1);
		Q_veh = new Matrix(SS,SS);
		Gam_veh = new Matrix(SS,SS);
		GamQGamT = new Matrix(NN,NN);
		Phi_veh = new Matrix(SS,SS);
		stateCovariance_veh = new Matrix(SS,SS);
		Pvl = new Matrix(SS,LL);
	}

	/**
	 * Initialize state vector from robot and landmarks.
	 * @param initialPose initial robot pose
	 * @param landmarks list of landmarks (with their positions inside them).
	 */
	public void setInitialState(Pose2D initialPose, ArrayList<RigidBodySprite> landmarks){
		// TODO: Fill me in
		state.setElement(XX, 0, initialPose.getX());
		state.setElement(YY, 0, initialPose.getY());
		state.setElement(TT, 0, initialPose.getTh());
		for (int i = 0; i < numLandmarks; i++) {
			state.setElement(SS + i * 2, 0, landmarks.get(i).poseContext().poseRelative.getX());
			state.setElement(SS + i * 2 + 1, 0, landmarks.get(i).poseContext().poseRelative.getY());
		}
	}
	
	/**
	 * Set the initial velocity to the desired value. This is useful if the robot starts
	 * with nonzero velocity. The alternative is to have a large Q value or a large rejction
	 * gate value and neither of these options is ideal.
	 * @param speed the initial speed.
	 */
	public void setInitialVelocity(double speed){
		state.setElement(VV,0,speed);
	}
	
	/**
	 * Sets the initial uncertainties of the states
	 * @param landmarkVariances the initial landmark uncertainties
	 */
	public void setInitialStateCovariance(double[] landmarkVariances){
		// TODO: Fill me in
		stateCovariance.setElement(0, 0, P0xx);
		stateCovariance.setElement(1, 1, P0yy);
		stateCovariance.setElement(2, 2, P0yy);
		stateCovariance.setElement(3, 3, P0vv);
		stateCovariance.setElement(4, 4, P0bb);
		if (numLandmarks > 0) {
			for (int i = 0; i < numLandmarks; i++) {
				stateCovariance.setElement(SS + i * 2, SS + i * 2, landmarkVariances[i]);
				stateCovariance.setElement(SS + i * 2 + 1, SS + i * 2 + 1, landmarkVariances[i]);
			}
		}
	}

	/**
	 * Makes sure no landmark covariance is less than the lower bound specified. For
	 * preventing landmarks from getting stuck forever.
	 * @param lowerBound
	 */
	public void trimLandmarkCovariance(double lowerBound){
   		if(doSLAM){
	   		for(int i=0 ; i<numLandmarks ; i++){
	   			int index = SS+2*i+0;
	   			double variance = stateCovariance.getElement(index,index);
	   			if(variance > 1e-12 && variance < lowerBound)
	   				stateCovariance.setElement(index,index,lowerBound);	
	   			index = SS+2*i+1;
	   			variance = stateCovariance.getElement(index,index);
	   			if(variance > 1e-12 && variance < lowerBound)
	   				stateCovariance.setElement(index,index,lowerBound);	
	   		}
   		}
	}
	
 	/**
 	 * Projects the system state and its covariance forward in time. 
 	 * @param dt time step
 	 */
 	public void kalmanSystemModel(double dt){
 		kalmanSystemModelSlow(dt);
 	}
 	/**
 	 * Projects the system state and its covariance forward in time. This version
 	 * does not partition the covariance calulations for SLAM so it is much slower for
 	 * large numbers of landmarks.
 	 * @param dt time step
 	 */
 	public void kalmanSystemModelSlow(double dt){
 		
		// TODO: Fill me in
 		double cos =Math.cos(state.getElement(2, 0));
 		double sin =Math.sin(state.getElement(2, 0));
 		Phi_veh.setDiagonal(1);
 		Phi_veh.setElement(0, 3, cos * dt);
 		Phi_veh.setElement(1, 3, sin * dt);
 		Phi_veh.setElement(2, 4, dt);
 		//slam matrix(NN*NN)
 		Matrix PhiSlam = new Matrix(NN,NN);
 		PhiSlam.setDiagonal(1);
 		PhiSlam.setSubMatrix(0, 0, Phi_veh);
 		//update state vector
 		Matrix.multiply(state, PhiSlam, state);
 		double new_angle = Angle.normalize(state.getElement(2, 0));
 		state.setElement(2, 0, new_angle);
 		//update state covariance
 		Matrix Phi_k = new Matrix(NN,NN);
 		double speed =state.getElement(3, 0);
 		Phi_k.setDiagonal(1);
 		Phi_k.setElement(0, 2, -speed * sin * dt);
 		Phi_k.setElement(0, 3, cos * dt);
 		Phi_k.setElement(1, 2, speed * cos * dt);
 		Phi_k.setElement(1, 3, sin * dt);
 		Phi_k.setElement(2, 4, dt);
 		//set Q matrix
 		Q_veh.setElement(0, 0, Qxx);
 		Q_veh.setElement(1, 1, Qyy);
 		Q_veh.setElement(2, 2, Qtt);
 		Q_veh.setElement(3, 3, Qvv);
 		Q_veh.setElement(4, 4, Qbb);
		Q_veh.scale(dt);
		Matrix QS = new Matrix(NN,NN);
		QS.setSubMatrix(0, 0, Q_veh);
		//set G matrix
		Gam_veh.setDiagonal(1);
		Gam_veh.setElement(0, 0, cos);
		Gam_veh.setElement(0, 1, -sin);
		Gam_veh.setElement(1, 0, sin);
		Gam_veh.setElement(1, 1, cos);
		Matrix GamS = new Matrix(NN,NN);
		GamS.setDiagonal(1);
		GamS.setSubMatrix(0, 0, Gam_veh);
		Matrix tmp = Matrix.multiply3(Phi_k, stateCovariance, Phi_k.transpose());
		Matrix tmp2 = Matrix.multiply3(GamS, QS, GamS.transpose());
		Matrix.add(stateCovariance, tmp, tmp2);
 	}
	 	
 	/**
 	 * Processes measurements of any kind to update the state
 	 * @param name sensor name (for debug printout)
 	 * @param sourceId the id of the associated data source (used only for printing debug info)
 	 * @param dz innovation
 	 * @param H measurement Jacobian
 	 * @param R measurement covariance
 	 * @param gateValue validation gate for rejecting outliers
 	 * @param doDebug print matrices if on
 	 */
  	public void processMeasurement(String name,int sourceId, Matrix dz,Matrix H, Matrix R, double gateValue, boolean doDebug){		
		// TODO: Fill me in
  		Matrix S = Matrix.multiply3(H, stateCovariance, H.transpose());
  		S.add(R);
  		Matrix MD_sq = Matrix.multiply3(dz.transpose(), S.inverse(), dz);
  		double MD = Math.sqrt(MD_sq.getElement(0, 0));
  		if (MD < gateValue){
  	  		Matrix S_n = Matrix.multiply3(H, stateCovariance, H.transpose());
  	  		S_n.add(R);
  	  		Matrix K = Matrix.multiply3(stateCovariance, H.transpose(), S_n.inverse());
  	  		Matrix tmp = Matrix.multiply(K, dz);
  	  		state.add(tmp);
  	  		double new_angle = Angle.normalize(state.getElement(2, 0));
  	  		state.setElement(2, 0, new_angle);
  	  		Matrix P = Matrix.multiply3(K, H, stateCovariance);
  	  		stateCovariance.sub(P);
  	  	}
  	}
  	
  	/**
  	 * Report on the content of a specific innovation type. This version does
  	 * not distinguish based on sensorModifier (used for landmark number).
  	 */
  	@SuppressWarnings("unused")
	private void reportInnovation(String sensorName, String targetSensorName,
  			double mahalanobisDistance, double dzMag){
  		if(sensorName == targetSensorName) 
  			System.out.printf("%s,%f,%f\n",sensorName, mahalanobisDistance,dzMag);
  	}
  	/**
  	 * Report on the content of a specific innovation type
  	 */
  	@SuppressWarnings("unused")
	private void reportInnovation(String sensorName, int sensorModifier, 
  			String targetSensorName, int targetSensorModifier, 
  			double mahalanobisDistance, double dzMag){
  		if(sensorName == targetSensorName && sensorModifier == targetSensorModifier) 
  			System.out.printf("%s,%d,%f,%f\n",sensorName, sensorModifier, mahalanobisDistance,dzMag);
  	}
}
