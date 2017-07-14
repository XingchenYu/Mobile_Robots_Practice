package applications.homeworks.unlocked.hw0;

import smrde.math.Matrix;

/**
 * The interface between a speed sensor and a Kalman filter. Provided so that the filter code is 
 * independent of the sensors used. Computes the innovation, the measurement Jacobian and the 
 * measurement covariance.
 *
 */
public class KalmanSensorSpeed {
	/**
	 * Controls debug printouts
	 */
	private boolean doDebug;
	/**
	 * State vector reference.
	 */
	private Matrix state;
	/**
	 * Speed measurement variance coefficient
	 */
	private double alphaV;
	/**
	 * Position of speed in the state vector
	 */
	private int speedStateIndex;
	/**
	 * Position of angular velocity in the state vector
	 */
	private int omegaStateIndex;
	/**
	 * Length of state vector
	 */
	private int numStates;
	/**
	 * Constructs a KalmanSensorSpeed
	 * @param state state vector
	 * @param doDebug flag to control printouts
	 * @param alphaV speed variance coefficient
	 * @param speedStateIndex position of linear velocity in the state vector
	 * @param omegaStateIndex position of angular velocity in the state vector
	 * @param numStates length of state vector
	 */
	public KalmanSensorSpeed(Matrix state, boolean doDebug, double alphaV, int speedStateIndex, 
			int omegaStateIndex, int numStates){
		this.state = state;
		this.doDebug = doDebug;
	 	this.alphaV = alphaV;
	 	this.speedStateIndex = speedStateIndex;
	 	this.omegaStateIndex = omegaStateIndex;
	 	this.numStates = numStates;
	}
	/**
	 * Compute the innovation.
	 * @param speedMeasurement the speed measurement
	 * @return the innovation vector z - h(x)
	 */
	public Matrix getInnovation(double speedMeasurement){
		Matrix dz = new Matrix(1,1);
		double act = state.getElement(speedStateIndex, 0);
		double diff = speedMeasurement - act;
		dz.setElement(0, 0, diff);
		//TODO: fill me in
		return dz;
	}
	/**
	 * Compute the measurement Jacobian
	 * @return the measurement Jacobian
	 */
	public Matrix geMeasurementJacobian(){
  		Matrix H = new Matrix(1,numStates);
  		//TODO: fill me in
  		//speed
  		H.setElement(0, speedStateIndex, 1);
 		return H;
	}
	/**
	 * Compute the measurement covariance.
	 * @param speedMeasurement speed measurement
	 * @param dt elapsed time since last call to this routine.
	 * @return the measurement covariance.
	 */
	public Matrix geMeasurementCovariance(double speedMeasurement,double dt){
  		Matrix R = new Matrix(1,1);
		//TODO: fill me in
  		double r = alphaV * Math.abs(speedMeasurement) * dt;
  		R.setElement(0, 0, r);
   		return R;
	}
}
