package applications.homeworks.unlocked.hw6;

/**
 * A class for 2D points in homogeneous coordinates. The interface is 
 * that of a 2D point but the extra 1 is carried around internally. Designed
 * to work with HT2D.
 * @author Xingchen Yu
 *
 */
public class Pt_2D{
	public double[] data = new double[3];
	
	/**
	 * Construct a new pose 2D and initialize it to the indicated values. Sets
	 * the scale factor (3rd elements) to 1.
	 * @param x desireed x coordinate
	 * @param y desired y coordinate
	 */
	public Pt_2D(double x, double y){
		data[0] = x;
		data[1] = y;
		data[2] = 1.0;
	}
	
	/**
	 * Construct a new pose 2D and initialize it to the indicated values.
	 * @param x desireed x coordinate
	 * @param y desired y coordinate
	 * @param w desired scale factor
	 */
	public Pt_2D(double x, double y, double w){
		data[0] = x;
		data[1] = y;
		data[2] = w;
	}
	
	public double x(){ return data[0];}
	public double y(){ return data[1];}
	public double w(){ return data[2];}
	
	/**
	 * Returns a convenient String for printing the contents of this.
	 * @return the convenient string.
	 */
	public String toString()
	{
		String str = String.format("x:%-7.4f y:%-7.4f th:%-7.4f",data[0], data[1], data[2]);
		return str;
	}
}
