package applications.homeworks.unlocked.hw0;

import smrde.math.Matrix;

/**
 * The interface between a gyro and a Kalman filter. Provided so that the filter code is independent 
 * of the sensors used. Computes the innovation, the measurement Jacobian and the measurement 
 * covariance.
 *
 */
public class KalmanSensorGyro {
	/**
	 * Controls debug printouts
	 */
	private boolean doDebug = false;
	/**
	 * State vector reference.
	 */
	private Matrix state;
	/**
	 * Gyro measurement variance coefficient
	 */
	private double sigmaGG;
	/**
	 * Position of angular velocity in the state vector
	 */
	private int omegaStateIndex;
	/**
	 * Length of state vector
	 */
	private int numStates;
	/**
	 * Constructs a KalmanSensorGyro
	 * @param state state vector
	 * @param doDebug flag to control printouts
	 * @param sigmaGG gyro variance
	 * @param omegaStateIndex position of angular velocity in the state vector
	 * @param numStates length of state vector
	 */
	public KalmanSensorGyro(Matrix state, boolean doDebug, double sigmaGG, int omegaStateIndex, int numStates){
		this.state = state;
		this.doDebug = doDebug;
	 	this.sigmaGG = sigmaGG;
	 	this.omegaStateIndex = omegaStateIndex;
	 	this.numStates = numStates;
	}
	/**
	 * Compute the innovatioon.
	 * @param omegaMeasurement the angular velocity measurement
	 * @return the innovation vector z - h(x)
	 */
	public Matrix getInnovation(double omegaMeasurement){
		Matrix dz = new Matrix(1,1);
		//TODO: fill me in
		double act = state.getElement(omegaStateIndex, 0);
		double diff = omegaMeasurement - act;
		dz.setElement(0, 0, diff);
   		return dz;
	}
	/**
	 * Compute the measurement Jacobian
	 * @return the measurement Jacobian
	 */
	public Matrix geMeasurementJacobian(){
  		Matrix H = new Matrix(1,numStates);
		//TODO: fill me in
  		H.setElement(0, omegaStateIndex, 1);
 		return H;
	}
	/**
	 * Compute the measurement covariance.
	 * @param omegaMeasurement angular velocity  measurement
	 * @param dt elapsed time since last call to this routine.
	 * @return the measurement covariance.
	 */
	public Matrix geMeasurementCovariance(double omegaMeasurement,double dt){
  		Matrix R = new Matrix(1,1);
		//TODO: fill me in
  		double r = sigmaGG * dt;
  		R.setElement(0, 0, r);
   		return R;
	}
}
