package applications.homeworks.unlocked.hw3;
import smrde.math.*;

/**
 * A class used to represent a WHEEL constraint between two rigid 
 * bodies. Such a constraint requires that the velocity in the y
 * direction of the body frame vanish for all time.
 * 
 * @author alonzo
 */
public class WMRWheelConstraint extends WMRDynamicsConstraint{
	
	private WMRDynamicsBody body;
	private final int dimension = 1;
	private final int numBodies = 1;
	private Matrix JacobianScratch;
	private Matrix FdDriftFreeScratch;
	private Matrix FdConsScratch;
	
	/**
	 * Constructs a WheelConstraint 
	 * @param body the body representing the wheel
	 * @param stateDim the length of the state vector
	 */
	public WMRWheelConstraint(WMRDynamicsBody body, int stateDim){
		this.body  = body;
		
		JacobianScratch = new Matrix(dimension,stateDim);
		FdDriftFreeScratch = new Matrix(dimension,1);
		FdConsScratch = new Matrix(dimension,1);
	}
	
	public int dimension() { return dimension;}
	public int numBodies() { return numBodies;}
	public int bodyIndex(int bodyNo){
		if(bodyNo ==0) return body.getBodyIndex();
		else return -1;
	}
	public Matrix JacobianScratch() { return JacobianScratch;}
	

	/**
	 * Computes the constraint Jacobian (CqDot) for this constraint
	 * @param x system state vector
	 * @param xd system state rate vector
	 * @return constraint Jacobian CqDot
	 */
	public Matrix Jacobian(Matrix x, Matrix xd){	
		Pose2D bodyPose = body.readBodyPose(x);
		// TODO Fill me in
		// use bodyPose.getTh()
		// use Math.sin() and Math.cos()
		// use JacobianScratch.setElement(0,whatever) three times
		// Always write in Row 0 of the Jacobian.
		// use body.getBodyIndex()+0 or 1 or 2 to index into 
		// the correct columns for the body
		double th = bodyPose.getTh();
		double cth = Math.cos(th);
		double sth = Math.sin(th);
		JacobianScratch.setElement(0,body.getBodyIndex(),-sth);
		JacobianScratch.setElement(0,body.getBodyIndex()+1,cth);
		JacobianScratch.setElement(0,body.getBodyIndex()+2,0);
    	return JacobianScratch;
    	
	}
	/**
	 * Computes the pseudoforce for this constraint without compensation for
	 * drift.
	 * @param x system state vector
	 * @param xd system state rate vector
	 * @return drift free pseudoforce
	 */
	public Matrix Fd(Matrix x, Matrix xd){		
		Pose2D bodyPose = body.readBodyPose(x);
		double th = bodyPose.getTh();
		double cth = Math.cos(th);
		double sth = Math.sin(th);
		
		PoseVector2D poseRate =  body.readBodyRate(xd);		
    	Vector2D Vi = new Vector2D(poseRate.x,poseRate.y);
    	//System.out.printf("Speed %f\n",Vi.magnitude());
    	
    	Vector2D xHat = new Vector2D(cth,sth);
    	double Q = 0.0;
    	
		// TODO Fill me in
    	// Compute the vector F
    	// Use Vector2D.dot for a dot product
    	// Use poseRate.th
    	double d_r = Vector2D.dot(Vi, xHat);
    	double thrate = poseRate.th;
    	Q = d_r * thrate;
    	FdDriftFreeScratch.setElement(0,0,Q);	
    	
    	return FdDriftFreeScratch;
	}
	/**
	 * Computes the drift compensating pseudoforce for this constraint.
	 * @param x system state vector
	 * @param xd system state rate vector
	 * @return drift pseudoforce
	 */
	public Matrix FdDrift(Matrix x, Matrix xd){		
		Pose2D bodyPose = body.readBodyPose(x);
		double th = bodyPose.getTh();
		double cth = Math.cos(th);
		double sth = Math.sin(th);
    	
		PoseVector2D poseRate = body.readBodyRate(xd);
    	Vector2D Vi = new Vector2D(poseRate.x,poseRate.y);
    	
    	Vector2D yHat = new Vector2D(-sth,cth);
    	double res = Vector2D.dot(Vi, yHat);

    	//System.out.printf("Wheel res: %f\n",res);
    	
    	FdConsScratch.setElement(0,0,-nholo_Kp*res);
    	return FdConsScratch;
	}
}
