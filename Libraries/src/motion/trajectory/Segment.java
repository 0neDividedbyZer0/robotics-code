package motion.trajectory;

public class Segment {
	protected double vel, pos, acc, jerk, dt, x, y, heading;
	
	public Segment(double vel, double pos, double acc, double jerk, double dt, double x, double y, double heading) {
		this.vel = vel;
		this.pos = pos;
		this.acc = acc;
		this.jerk = jerk;
		this.dt = dt;
		this.x = x;
		this.y = y;
		this.heading = heading;
	}
	
	public Segment() {
		
	}

}
