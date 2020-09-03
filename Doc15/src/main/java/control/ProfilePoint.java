package control;

/**
 * 
 * @author Maverick Zhang
 * This class is designed to hold points for the Trapezoidal Profile class. 
 * It simply holds your position and velocity data at a point.
 *
 */
public class ProfilePoint {

	protected double pos, vel;
	
	public ProfilePoint(double pos, double vel) {
		this.pos = pos;
		this.vel = vel;
	}
	
	public double getPos() {
		return pos;
	}
	
	public double getVel() {
		return vel;
	}

}
