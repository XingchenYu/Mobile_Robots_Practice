package applications.homeworks.unlocked.hw3;
import java.awt.geom.Point2D;

import smrde.math.*;

/**
 * A class used to represent a ROTARY constraint between two rigid 
 * bodies. Such a constraint requires that both participating bodies
 * agree on the position of a distinguished point.
 * 
 * @author alonzo
 */
public class WMRRotaryConstraint extends WMRDynamicsConstraint{
	
	private WMRDynamicsBody bodyI;
	private WMRDynamicsBody bodyJ;
	private Pose2D ptToBodyI; // initial pose of Pt wrt Body I
	private Pose2D ptToBodyJ; // initial pose of Pt wrt Body J
	private final int dimension = 2;
	private final int numBodies = 2;
	private Matrix JacobianScratch;
	private Matrix FdDriftFreeScratch;
	private Matrix FdConsScratch;
	
	/**
	 * Constructs a RotaryConstraint
	 * @param bodyI first body
	 * @param bodyJ second body
	 * @param ptPose pose of a point to remain fixed wrt both bodies
	 * @param stateDim the length of the state vector
	 */
	public WMRRotaryConstraint(WMRDynamicsBody bodyI, WMRDynamicsBody bodyJ, 
			Pose2D ptPose, int stateDim){
		
		this.bodyI = bodyI;
		this.bodyJ = bodyJ;
			
		this.ptToBodyI = bodyI.getBodyPose().inverse(); // world to Body i
		this.ptToBodyJ = bodyJ.getBodyPose().inverse(); // world to Body j
		
		this.ptToBodyI.concatenate(ptPose);
		this.ptToBodyJ.concatenate(ptPose);
		
		JacobianScratch = new Matrix(dimension,stateDim);
		FdDriftFreeScratch = new Matrix(dimension,1);
		FdConsScratch = new Matrix(dimension,1);
	}
	
	public int dimension() { return dimension;}
	public int numBodies() { return numBodies;}
	public int bodyIndex(int bodyNo){
		if(bodyNo ==0) return bodyI.getBodyIndex();
		else if(bodyNo ==1) return bodyJ.getBodyIndex();
		else return -1;
	}
	public Matrix JacobianScratch() { return JacobianScratch;}

	/**
	 * Computes the constraint Jacobian (CxDot) for this constraint
	 * @param x system state vector
	 * @param xd system state rate vector
	 * @return constraint Jacobian CxDot
	 */
	public Matrix Jacobian(Matrix x, Matrix xd){		
		Pose2D bodyIPose = bodyI.readBodyPose(x);
		Pose2D bodyJPose = bodyJ.readBodyPose(x);
		
		// Get world position of pt using both body poses
    	Point2D.Double ptPosnWrtBodyI = (Point2D.Double) this.ptToBodyI.getPosition();
    	Point2D.Double ptPosnWrtBodyJ = (Point2D.Double) this.ptToBodyJ.getPosition();
    	bodyIPose.deltaTransform(ptPosnWrtBodyI, ptPosnWrtBodyI);
    	bodyJPose.deltaTransform(ptPosnWrtBodyJ, ptPosnWrtBodyJ);
    	double dxi = ptPosnWrtBodyI.getX();
    	double dyi = ptPosnWrtBodyI.getY();
    	double dxj = ptPosnWrtBodyJ.getX();
    	double dyj = ptPosnWrtBodyJ.getY();
    	
		// TODO Fill me in
		// use JacobianScratch.setElement(0,whatever) four times
		// use JacobianScratch.setElement(1,whatever) four times
		// use bodyI.getBodyIndex()+0 or 1 or 2 to index into 
		// the correct columns for the body
    	// same for bodyJ
    	
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
		Pose2D bodyIPose = bodyI.readBodyPose(x);
		Pose2D bodyJPose = bodyJ.readBodyPose(x);
		
		// Get world position of pt using both body poses
    	Point2D.Double ptPosnWrtBodyI = (Point2D.Double) this.ptToBodyI.getPosition();
    	Point2D.Double ptPosnWrtBodyJ = (Point2D.Double) this.ptToBodyJ.getPosition();
    	bodyIPose.deltaTransform(ptPosnWrtBodyI, ptPosnWrtBodyI);
    	bodyJPose.deltaTransform(ptPosnWrtBodyJ, ptPosnWrtBodyJ);
    	double dxi = ptPosnWrtBodyI.getX();
    	double dyi = ptPosnWrtBodyI.getY();
    	double dxj = ptPosnWrtBodyJ.getX();
    	double dyj = ptPosnWrtBodyJ.getY();
		  
    	PoseVector2D poseRateI = bodyI.readBodyRate(xd);
    	PoseVector2D poseRateJ = bodyJ.readBodyRate(xd);
    	
    	double Wi = poseRateI.th;
    	double Wj = poseRateJ.th;
    	
		// TODO Fill me in
    	// COmpute Qdx and Qdy and put them in QdDriftScratch
    	
    	return FdDriftFreeScratch;
	}
	/**
	 * Computes the drift compensating pseudoforce for this constraint.
	 * @param x system state vector
	 * @param xd system state rate vector
	 * @return drift pseudoforce
	 */ 
	public Matrix FdDrift(Matrix x, Matrix xd){
		Pose2D bodyIPose = bodyI.readBodyPose(x);
		Pose2D bodyJPose = bodyJ.readBodyPose(x);
		
		bodyIPose.concatenate(this.ptToBodyI);	
		bodyJPose.concatenate(this.ptToBodyJ);
		
		FdConsScratch.setElement(0, 0, -holo_Kp*(bodyIPose.getX()-bodyJPose.getX()));
		FdConsScratch.setElement(1, 0, -holo_Kp*(bodyIPose.getY()-bodyJPose.getY()));
		
		//FdConsScratch.print("Rotary", "%f5.3", true);

    	return FdConsScratch;
	}
}
