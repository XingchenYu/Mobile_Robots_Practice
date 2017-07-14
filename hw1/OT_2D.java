package applications.homeworks.unlocked.hw6;

/**
 * A class for orthogonal transforms in 2D. 
 * 
 * Rename this class to RIGID transforms next time around.
 * 
 * @author Xingchen Yu
 *
 */
public class OT_2D{
	public double[][] data = new double[3][3];

	/**
	 * Constructs a new OT_2D and initializes it to the identity matrix
	 */
	public OT_2D(){
		data[0][0] = 1.0;
		data[1][1] = 1.0;
		data[2][2] = 1.0;
	}
	
	/**
	 * Constructs a new OT_2D and initializes its first two rows to the
	 * provided values. Sets the last row to {0 0 1}
	 */
	public OT_2D(double d00, double d01, double d02, double d10, double d11, double d12){
		data[0][0] = d00;
		data[0][1] = d01;
		data[0][2] = d02;
		data[1][0] = d10;
		data[1][1] = d11;
		data[1][2] = d12;
		data[2][2] = 1.0;
	}
	
	/**
	 * Constructs a new OT_2D which is a copy of the provided one. This is a
	 * "deep" copy, not just a reference to its content.
	 */
	public OT_2D(OT_2D ht){
		set(ht);
	}

	/**
	 * Constructs a new OT_2D with the specified pose parameters.
	 * @param x x position
	 * @param y y position
	 * @param th orientation
	 */
	public OT_2D(double x, double y, double th) {
		set(x,y,th);
	}

	/**
	 * Set this to the value of the provided OT_2D
	 */
	public void set(OT_2D ht){
		for(int i=0 ; i<3 ; i++)
			{
			for(int j=0 ; j<3 ; j++)
				{
				data[i][j] = ht.data[i][j];
				}
			}
	}
	
	/**
	 * Set this to the value of the provided OT_2D
	 */
	public void set(double x, double y, double th){
		data[0][0] =  Math.cos(th);
		data[0][1] = -Math.sin(th);
		data[0][2] =  x;
		data[1][0] =  Math.sin(th);
		data[1][1] =  Math.cos(th);
		data[1][2] =  y;
		data[2][2] =  1.0;
	}
	
	/**
	 * Postmultiply this by the indicated OT_2D to produce a new OT_2D in this.
	 * @param ht
	 */
	@StudentFillIn ("Complete concat")
	public void concat(OT_2D ht){
		// Don't overwrite this while trying to read from it. Make
		// a read-only copy of this in temp to use while overwriting this.
		OT_2D temp = new OT_2D(this);
		
		for(int i=0 ; i<3 ; i++)
			{
			for(int j=0 ; j<3 ; j++)
				{
				double sum = 0.0;
				for(int k=0 ; k<3 ; k++)
					{
					sum += ht.data[i][k] * temp.data[k][j]; // fill me in
					}
				data[i][j] = sum;
				}
			}	
	}
	
	/**
	 * Compute the transpose of this. Can be useful for computing the 
	 * inverse.
	 */
	public OT_2D transpose(){
		// Make space for the output
		OT_2D temp = new OT_2D();
		
		for(int i=0 ; i<3 ; i++)
			{
			for(int j=0 ; j<3 ; j++)
				{
				temp.data[i][j] = data[j][i];
				}
			}
		return temp;
	}
	
	/**
	 * Transforms the indicated point with this and returns a new Pt2D which
	 * contains the result. This method simply calls transform(pt) and returns 
	 * its result.
	 * @param pt the point to transform
	 */
	@StudentFillIn ("Complete transform")
	public Pt_2D transform(Pt_2D pt){
		pt = this.operate(pt);
		return this.operate(pt); // fix me
	}
	
	/**
	 * Operate on the indicated point with this and return a new Pt2D which
	 * contains the result
	 * @param pt the point to operate on
	 */
	@StudentFillIn ("Complete operate")
	public Pt_2D operate(Pt_2D pt){
		Pt_2D temp = new Pt_2D(0.0,0.0);
		
		for(int i=0 ; i<3 ; i++)
			{
			double sum = 0.0;
			for(int j=0 ; j<3 ; j++)
				{
				sum += data[i][j] * pt.data[j]; // fill me in
				}
			temp.data[i] = sum;
			}	
		return temp;
	}
	
	/**
	 * Returns a new OT_2D which is the inverse of this.
	 * @return the inverse transform
	 */
	@StudentFillIn ("Complete inverse")
	public OT_2D inverse(){
		// make space for the result
		OT_2D temp = this.transpose();
		temp.data[2][0] = 0.0;
		temp.data[2][1] = 0.0;
		Pt_2D p = new Pt_2D(data[0][2],data[1][2]);
		Pt_2D p_t = temp.operate(p);
		temp.data[0][2] = -p_t.data[0];
		temp.data[1][2] = -p_t.data[1];
		// create a new Pt_2D, operate on it and put its
		// negative into temp.
		// add more here.
		return temp;
	}
	
	/**
	 * Construct a new HT which encodes the indicated translation operator and
	 * returns it.
	 * @param x the x coordinate of the desired translation
	 * @param y the y coordinate of the desired translation
	 * @return the translation operator
	 */
	public static OT_2D getTransInstance(double x, double y){
		OT_2D HT = new OT_2D();
		HT.data[0][2] = x;
		HT.data[1][2] = y;
		return HT;
	}
	
	/**
	 * Construct a new HT which encodes the indicated rotation operator and
	 * returns it.
	 * @param th the rotation angle of the desired OT_2D
	 * @return the rotation operator
	 */
	public static OT_2D getRotInstance(double th){
		OT_2D HT = new OT_2D();
		HT.data[0][0] =  Math.cos(th); 		HT.data[0][1] = -Math.sin(th);
		HT.data[1][0] =  Math.sin(th); 		HT.data[1][1] =  Math.cos(th);
		return HT;
	}
	
	/**
	 * Return the first (x) column of this.
	 * @return the unit vector in the first (x) column
	 */
	@StudentFillIn ("Complete XUnitVector")
	public Pt_2D XUnitVector(){
		Pt_2D pt = new Pt_2D(0.0,1.0,0.0); //use new Pt_2D(...);
		pt.data[0] = data[0][0];
		pt.data[1] = data[1][0];
		pt.data[2] = data[2][0];
		return pt;
	}
	
	/**
	 * Return the second (y) column of this.
	 * @return the unit vector in the second (y) column
	 */
	@StudentFillIn ("Complete YUnitVector")
	public Pt_2D YUnitVector(){
		Pt_2D pt = new Pt_2D(1.0,0.0,0.0); //use new Pt_2D(...);
		pt.data[0] = data[0][1];
		pt.data[1] = data[1][1];
		pt.data[2] = data[2][1];
		return pt;
	}
	
	/**
	 * Return the third (origin) column of this.
	 * @return the unit vector in the third (origin) column
	 */
	@StudentFillIn ("Complete OUnitVector")
	public Pt_2D OUnitVector(){
		Pt_2D pt = new Pt_2D(0.0,0.0,1.0); //use new Pt_2D(...);
		pt.data[0] = data[0][2];
		pt.data[1] = data[1][2];
		pt.data[2] = data[2][2];
		return pt;
	}
	
	@StudentFillIn ("Complete OUnitVector")
	// get the right element of data
	public double x(){ return data[0][2];} // fix me
	@StudentFillIn ("Complete OUnitVector")
	// get the right element of data
	public double y(){ return data[1][2];} // fix me
	@StudentFillIn ("Complete OUnitVector")
	// get the right elements of dataUse Math.atan2();
	public double th(){ return Math.atan2(data[1][0],data[0][0]);} // fix me
	
	/**
	 * Returns a convenient String for printing the contents of this. This
	 * three line output will also work in the debugger.
	 * @return the convenient string.
	 */
	public String toString()
	{
		String str = String.format("x:%-7.4f y:%-7.4f th:%-7.4f \n",data[0][0], data[0][1], data[0][2]);
		str = str +  String.format("x:%-7.4f y:%-7.4f th:%-7.4f \n",data[1][0], data[1][1], data[1][2]);
		str = str +  String.format("x:%-7.4f y:%-7.4f th:%-7.4f \n",data[2][0], data[2][1], data[2][2]);
		return str;
	}
}
