package applications.homeworks.unlocked.hw3;
import smrde.math.*;

/**
 * A class used to represent a RIGID constraint between two
 * rigid bodies. Such a constraint requires that the relative pose
 * between both participating bodies be constant.
 * 
 * @author alonzo
 *
 */
public class WMRRigidConstraint extends WMRDynamicsConstraint{
	
	private WMRDynamicsBody bodyI;
	private WMRDynamicsBody bodyJ;	
	
	private Pose2D originalBodyIToBodyJ;
	private final int dimension = 3;
	private final int numBodies = 2;
	
	private Matrix JacobianScratch;
	private Matrix FdDriftFreeScratch;
	private Matrix FdConsScratch;
	
	private Matrix residual;
	private Matrix iResidual;   // integrated
	private Matrix siResidual;  // scaled integrated
	
	/**
	 * Constructs a RigidConstraint
	 * @param bodyI first body
	 * @param bodyJ second body
	 * @param stateDim the length of the state vector
	 */
	public WMRRigidConstraint(WMRDynamicsBody bodyI, WMRDynamicsBody bodyJ, int stateDim){
		this.bodyI = bodyI;
		this.bodyJ = bodyJ;
		originalBodyIToBodyJ = relativePose(bodyI.getBodyPose(),bodyJ.getBodyPose());
		JacobianScratch = new Matrix(dimension,stateDim);
		FdDriftFreeScratch = new Matrix(dimension,1);
		FdConsScratch = new Matrix(dimension,1);
		
		residual = new Matrix(dimension,1);
		iResidual = new Matrix(dimension,1);
		siResidual = new Matrix(dimension,1);
	}
	
	public int dimension() { return dimension;}
	public int numBodies() { return numBodies;}
	public int bodyIndex(int bodyNo){
		if(bodyNo == 0) return bodyI.getBodyIndex();
		else if(bodyNo == 1) return bodyJ.getBodyIndex();
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
		Pose2D bodyIToBodyJ = relativePose(bodyIPose,bodyJPose);
    	//System.out.printf("BodyIToJPose %s\n",bodyIToBodyJ.toString());
    	
    	double th 	= bodyJPose.getTh();
    	double cth 	= Math.cos(th);
    	double sth 	= Math.sin(th);
    	
		// TODO Fill me in
		// use JacobianScratch.setElement(0,whatever) five times
		// use JacobianScratch.setElement(1,whatever) five times
		// use JacobianScratch.setElement(2,whatever) two times
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
	public Matrix Fd(Matrix x,Matrix xd){		

    	double th 	= x.getElement(bodyJ.getBodyIndex()+2, 0);
    	double cth 	= Math.cos(th);
    	double sth 	= Math.sin(th);
    	
    	PoseVector2D poseRateI = bodyI.readBodyRate(xd);
    	PoseVector2D poseRateJ = bodyJ.readBodyRate(xd);
    	
    	double W   = poseRateJ.th;
    	
    	double Vix = poseRateI.x;
    	double Viy = poseRateI.y;
    	double Vjx = poseRateJ.x;
    	double Vjy = poseRateJ.y;
    	
    	double Fdx = 0.0;
    	double Fdy = 0.0;
    	double Fdt = 0.0;
    	
		// TODO Fill me in
    	// COmpute Fdx, Fdy, and Fdt and put them in FdDriftScratch
    	
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
		Pose2D bodyIToBodyJ = relativePose(bodyIPose,bodyJPose);
		
		residual.setElement(0,0,(bodyIToBodyJ.getX() -originalBodyIToBodyJ.getX()));
		residual.setElement(1,0,(bodyIToBodyJ.getY() -originalBodyIToBodyJ.getY()));
		residual.setElement(2,0,(bodyIToBodyJ.getTh()-originalBodyIToBodyJ.getTh()));
		
		iResidual.add(residual);
		
		FdConsScratch.setMatrix(residual);
		FdConsScratch.scale(-holo_Kp);
		
//		Integral seems to add no value. MATLAB work said it does for wheelConstraint
//		siResidual.setMatrix(iResidual);
//		siResidual.scale(-holo_Kp/100.0);
//		QdConsScratch.add(siResidual);
		
    	return FdConsScratch;
	}
	
	private Pose2D relativePose(Pose2D bodyIPose,Pose2D bodyJPose){
    	Pose2D bodyIToBodyJ = bodyJPose.inverse(); // worldToBodyJ
    	bodyIToBodyJ.concatenate(bodyIPose);
    	return bodyIToBodyJ;
	}
}
