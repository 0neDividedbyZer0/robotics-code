package control;

/**
 * 
 * @author Maverick Zhang
 * This is a class that restricts profile generation to the robot's maximum kinematic constraints.
 * For instance, you cannot use a profile that goes to a higher speed than your robot's maximum speed.
 *
 */
public class ProfileConstraints {
	private double max_speed, max_acceleration;
	
	public ProfileConstraints(double max_speed, double max_acceleration) {
		this.max_speed = max_speed;
		this.max_acceleration = max_acceleration;
	}
	
	public double getMaxSpeed() {
		return max_speed;
	}
	
	public double getMaxAcceleration() {
		return max_acceleration;
	}
	
	public String toString() {
		return "Max speed: " + max_speed + " Max acceleration: " + max_acceleration;
	}

}
