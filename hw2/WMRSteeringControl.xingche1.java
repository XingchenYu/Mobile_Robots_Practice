package applications.homeworks.unlocked.hw3;

import smrde.math.Angle;
import smrde.math.Pose2D;
import smrde.objects.Bicycle;
import smrde.systems.RigidBodySprite;

/**
 * A class used to compute consistent steer angles for an automobile to avoid energy
 * loss and constraint instability associated with collapse of the constraint 
 * nullspace.
 * 
 * @author alonzo
 *
 */
public class WMRSteeringControl {
	
	private static final double gain = 3.0;
	
	/**
	 * Controls the steer angle with a proportional controller. A force is generated
	 * and applied which is proportional to the angle error.
	 * @param body the vehicle to be steered
	 * @param desiredSteerAngle the desired steer angle
	 * @param wheel the wheel to steer
	 */
    public static void controlSteerAngle(RigidBodySprite body, double desiredSteerAngle, RigidBodySprite wheel){
    	double bodyTh  = body.poseContext().poseRelative.getTh();
    	double wheelTh = wheel.poseContext().poseRelative.getTh();
    	double actualSteerAngle = wheelTh-bodyTh;
    	actualSteerAngle = Angle.normalize(actualSteerAngle);
    	double angleError = desiredSteerAngle-actualSteerAngle;
    	angleError = Angle.normalize(angleError);
    	double steerForce = gain*(angleError);
    	wheel.inertialContext().force.th  = steerForce;
    	//System.out.printf("angleError: %f deltaTime: %f\n",angleError, deltaTime);
    	//System.out.printf("ForceTemp: %f\n",steerForce);
    }
    
    /**
     * Sets the steer angles of the car directly. Used to set up initial conditions.
     * @param alpha the steer angle of an equivalent bicycle
     * @param leftFrntPose pose of left front wheel (modified here)
     * @param rghtFrntPose pose of right front wheel (modified here)
     */
    public static void setCarSteering(double alpha, Pose2D leftFrntPose, Pose2D rghtFrntPose){	
    	double beta1 = consistentLeftSteerAngle(alpha);
		leftFrntPose.setPose(leftFrntPose.getX(), leftFrntPose.getY(), beta1); // steer front wheel
    	double beta2 = consistentRghtSteerAngle(alpha);
		rghtFrntPose.setPose(rghtFrntPose.getX(), rghtFrntPose.getY(), beta2); // steer rear wheel
    }
    
    /**
     * Compute the angle of the left wheel which is consistent with the provided 
     * equivalent bicycle angle
     * @param alpha the steer angle of an equivalent bicycle
     * @return the consistent left steer angle
     */
    public static double consistentLeftSteerAngle(double alpha){
		double L = Bicycle.leftFrontPose().getX()-Bicycle.leftRearPose().getX();
		double W = Bicycle.leftRearPose().getY() -Bicycle.rghtRearPose().getY();
		double R = L/Math.tan(alpha);
		// answer for negative alpha is off by PI without signum fix
		double sgn = Math.signum(alpha);
		double beta = 0.0;
		// TODO: Fill me in
		// compute beta
		//System.out.printf("alpha1: %f beta1: %f\n",alpha, beta);
		beta = sgn * Math.atan(sgn * L / ((R - W) / 2));
		return beta;
    }
    
    /**
     * Compute the angle of the right wheel which is consistent with the provided 
     * equivalent bicycle angle
     * @param alpha the steer angle of an equivalent bicycle
     * @return the consistent right steer angle
     */
    public static double consistentRghtSteerAngle(double alpha){
		double L = Bicycle.leftFrontPose().getX()-Bicycle.leftRearPose().getX();
		double W = Bicycle.leftRearPose().getY() -Bicycle.rghtRearPose().getY();
		double R = L/Math.tan(alpha);
		// answer for negative alpha is off by PI without signum fix
		double sgn = Math.signum(alpha);
		double beta = 0.0;
		// TODO: Fill me in
		// compute beta
		//System.out.printf("alpha2: %f beta2: %f\n",alpha, beta);
		beta = sgn * Math.atan(sgn * L / ((R + W) / 2));
		return beta;
    }
}
