package applications.homeworks.unlocked.hw6;

/**
 * A pose (x,y,th) for objects in 2D. 
 * @author Xingchen Yu
 *
 */
public class Pose_2D extends OT_2D{
	/**
	 * Constructs and initializes a Pose to (0,0,0).
	 */
	public Pose_2D(){
		super();
	}
	/**
	 * Constructs and initializes a Pose to (x,y,th).
	 * @param x
	 * @param y
	 * @param th
	 */
	public Pose_2D(double x, double y, double th){
		super(x,y,th);
	}
	/**
	 * Sets the pose to the designated value
	 * @param x
	 * @param y
	 * @param th
	 */
	public void setPose(double x, double y, double th)
	{
		super.set(x,y,th);
	}
	/**
	 * Use this to print a pose with:
	 * 
	 * System.out.println(toString(myPose));
	 */
	public String toString()
	{
		String str = String.format("x: %6.3f y:%6.3f th:%6.3f", x(),y(),th());
		return str;
	}
	/**
	 * Returns the inverse of the present pose
	 */
	@StudentFillIn ("Complete inverse")
	public Pose_2D inverse()
	{
		
		OT_2D temp = super.inverse();
		Pose_2D invPose = new Pose_2D(temp.x(),temp.y(),temp.th());
		
		System.out.println(temp.x());
		System.out.println(temp.y());
		System.out.println(temp.th());
		return(invPose);
	}
}
