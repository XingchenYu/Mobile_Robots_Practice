package applications.homeworks.unlocked.hw3;

import smrde.math.Matrix;
import smrde.math.Pose2D;

/**
 * A class to perform forward and inverse kinematics for a very general automobile 
 * with wheels that are offset from their steer axes.
 * 
 * @author alonzo
 *
 */
public class WMRKinematics {
	
	/**
	 * Compute the wheel angles of the indicated wheel velocities and return them in 
	 * the wheel angle vector.
	 * @param thWheel wheel angle vector
	 * @param vc wheel velocities
	 */
	public static void computeDesiredWheelAngles(Matrix thWheel, Matrix vc){
		// TODO Fill me in
		// Use Math.atan2() to get the angles
		// Set the angles with calls to thWheel.setElement(i,j,value);
		double v1x = vc.getElement(0,0);
		double v1y = vc.getElement(1,0);
		double v2x = vc.getElement(2,0);
		double v2y = vc.getElement(3,0);
		double v3x = vc.getElement(4,0);
		double v3y = vc.getElement(5,0);
		double v4x = vc.getElement(6,0);
		double v4y = vc.getElement(7,0);
		double angle1 =  Math.atan2(v1y, v1x);
		double angle2 =  Math.atan2(v2y, v2x);
		double angle3 =  Math.atan2(v3y, v3x);
		double angle4 =  Math.atan2(v4y, v4x);
		thWheel.setElement(0,0,angle1);
		thWheel.setElement(1,0,angle2);
		thWheel.setElement(2,0,angle3);
		thWheel.setElement(3,0,angle4);
    }
    
	/**
	 * Converts the velocities of the steer axes to velocity of the vehicle expressed 
	 * in body coordinates.
	 * @param VBody velocity of the vehicle expressed in body coordinates (output)
	 * @param Hs Jacobian matrix 
	 * @param vs steer axes velocities in body coordinates
	 */
    public static void steerAxisToBodyVelocity(Matrix VBody,Matrix Hs, Matrix vs){
    	// TODO Fill me in
    	// use a Matrix.multiply();
    	// also see Matrix.generalizedInverse() in the javadocs
    	Matrix tmp = Hs.generalizedInverse();
    	Matrix.multiply(VBody, tmp, vs);
    }
    
    /**
     * The 8 X 3 Jacobian relating the wheel axis velocities to the vehicle velocity
     * @param Hs Jacobian (output)
     * @param W vehicle width
     * @param L vehicle length
     */
    public static void setSteerAxisJacobian(Matrix Hs, double W, double L){
    	// TODO Fill me in
    	// use Hs.setElement(i,j,value) for each nonzero value;
    	Hs.setElement(0,0,1);
    	Hs.setElement(1,1,1);
    	Hs.setElement(2,0,1);
    	Hs.setElement(3,1,1);
    	Hs.setElement(4,0,1);
    	Hs.setElement(5,1,1);
    	Hs.setElement(6,0,1);
    	Hs.setElement(7,1,1);
    	Hs.setElement(0,2,-W/2);
    	Hs.setElement(1,2,L/2);
    	Hs.setElement(2,2,W/2);
    	Hs.setElement(3,2,L/2);
    	Hs.setElement(4,2,-W/2);
    	Hs.setElement(5,2,-L/2);
    	Hs.setElement(6,2,W/2);
    	Hs.setElement(7,2,-L/2);
    }
    
    /**
     * The 8 X 5 Jacobian relating the increments to wheel velocities that are due to steer 
     * rates to the angular velocity and steer rates.
     * @param Hc Jacobian (output)
     * @param d wheel offset (length of steering boom)
     * @param thWheel vector of wheel steer angles
     */
    public static void setWheelJacobian(Matrix Hc, double d, Matrix thWheel){
    	// TODO Fill me in
    	// Use Math.sin() and Math.cos()
    	// Use Hc.setElement(i,j,value);
    	double a1 = -d * Math.sin(thWheel.getElement(0,0));
    	double b1 = d * Math.cos(thWheel.getElement(0,0));
    	double a2 = d * Math.sin(thWheel.getElement(1,0));
    	double b2 = -d * Math.cos(thWheel.getElement(1,0));
    	double a3 = -d * Math.sin(thWheel.getElement(2,0));
    	double b3 = d * Math.cos(thWheel.getElement(2,0));
    	double a4 = d * Math.sin(thWheel.getElement(3,0));
    	double b4 = -d * Math.cos(thWheel.getElement(3,0));
    	
    	Hc.setElement(0,0,-b1);
    	Hc.setElement(0,1,-b1);
    	Hc.setElement(1,0,a1);
    	Hc.setElement(1,1,a1);
    	Hc.setElement(2,0,-b2);
    	Hc.setElement(2,2,-b2);
    	Hc.setElement(3,0,a2);
    	Hc.setElement(3,2,a2);
    	Hc.setElement(4,0,-b3);
    	Hc.setElement(4,3,-b3);
    	Hc.setElement(5,0,a3);
    	Hc.setElement(5,3,a3);
    	Hc.setElement(6,0,-b4);
    	Hc.setElement(6,4,-b4);
    	Hc.setElement(7,0,a4);
    	Hc.setElement(7,4,a4);
    }
    
    /**
     * Compute wheel velocities from the velocities of the steer axes.
     * @param vcw wheel contact point velocities (output)
     * @param vs wheel steer axis velocities
     * @param Hc wheel Jacobian
     * @param VcWorld wheel contact point velocities
     */
    public static void steerAxisToWheelVelocity(Matrix vcw,Matrix vs,Matrix Hc,Matrix VcWorld){
		// TODO Fill me in
    	// Use Matrix.multiply()
    	// Use Matrix.add()
    	Matrix.add(vcw, vs, Matrix.multiply(Hc, VcWorld));
	}
	
    /**
     * Compute steer axis velocities from wheel velocities.
//     * @param vcw wheel contact point velocities
     * @param vs wheel steer axis velocities (output)
     * @param Hc wheel Jacobian
     * @param VcWorld wheel contact point velocities
     */
    public static void wheelToSteerAxisVelocity(Matrix vcw,Matrix vs,Matrix Hc,Matrix VcWorld){
		// TODO Fill me in
    	// Use Matrix.multiply()
    	// Use Matrix.sub()
    	Matrix.sub(vs, vcw, Matrix.multiply(Hc, VcWorld));
	}
	
    /**
     * Compute steer axis velocities from body velocities
     * @param v velocities of wheel steer axes
     * @param Hs steer axis Jacobian
     * @param VBody vehicle linear and angular velocity in body coordinates
     */
    public static void bodyToSteerAxisVelocity(Matrix v, Matrix Hs, Matrix VBody){
    	// TODO Fill me in
    	// use a Matrix.multiply();
    	Matrix.multiply(v, Hs, VBody);
    }
    
    /**
     * Convert linear and angular vehicle velocity from body coordinates to 
     * world coordinates.
     * @param bodyToWorld pose2D of body frame wrt world frame
     * @param bodyVelocity velocity in body coords
     * @param worldVelocity velocity in world coords(output)
     */
    public static void bodyToWorldVelocity(Pose2D bodyToWorld, Matrix bodyVelocity, Matrix worldVelocity){
    	double VxB = bodyVelocity.getElement(0,0);
    	double VyB = bodyVelocity.getElement(1,0);
    	double OmegaB = bodyVelocity.getElement(2,0);
    	double w = bodyToWorld.getTh();
    	double cw = Math.cos(w);
    	double sw = Math.sin(w);
    	double VxW=0;
    	double VyW=0;
    	// TODO fill in the actual transformation to 
    	// get VxW and VyW
    	VyW = VyB * cw + VxB * sw;
    	VxW = VxB * cw - VyB * sw;
    	double OmegaW = OmegaB;
    	worldVelocity.setElement(0,0,VxW);
    	worldVelocity.setElement(1,0,VyW);
    	worldVelocity.setElement(2,0,OmegaW);
    }
    
    /**
     * Convert linear and angular vehicle velocity from world coordinates to 
     * body coordinates.
     * @param bodyToWorld pose2D of body frame wrt world frame
     * @param worldVelocity velocity in world coords(output)
     * @param bodyVelocity velocity in body coords
     */
    public static void worldToBodyVelocity(Pose2D bodyToWorld, Matrix worldVelocity, Matrix bodyVelocity){
       	double VxW = worldVelocity.getElement(0,0);
    	double VyW = worldVelocity.getElement(1,0);
    	double OmegaW = worldVelocity.getElement(2,0);
    	double w = bodyToWorld.getTh();
    	double cw = Math.cos(w);
    	double sw = Math.sin(w);
    	double VxB = 0.0;
    	double VyB = 0.0;
    	// TODO fill in the actual transformation to 
    	// get VxB and VyB
    	VyB = VyW * cw - VxW * sw;
    	VxB = VxW * cw + VyW * sw;
    	double OmegaB = OmegaW;
    	bodyVelocity.setElement(0,0,VxB);
    	bodyVelocity.setElement(1,0,VyB);
    	bodyVelocity.setElement(2,0,OmegaB);
    }
}

