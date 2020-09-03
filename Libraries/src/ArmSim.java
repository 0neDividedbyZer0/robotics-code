
public class ArmSim {
	protected static final double freeSpeed = 18730.0 * (2.0 * Math.PI / 60.0); //radians per second
	protected static final double freeCurrent = 0.7; //Amperes
	protected static final double stallTorque = 0.71; //Newton-meters
	protected static final double stallCurrent = 134.0; //Amperes
	protected static final double R = 12.0 / stallCurrent;
	protected static final double k_t = stallTorque / stallCurrent;
	protected static final double k_v = (12.0 - freeCurrent * R) / freeSpeed;
	protected static final double G = 833.33;//(12.0 * 0.88 * 2.0)/ (k_v * Math.PI//
	protected static final double m = 15.0 * 0.454; //kg
	protected static final double r = 18.0 * 0.0254; //m
	protected static final double J = 0.5 * m * r * r;
	protected static final double a = (-k_v * k_t * G * G)/ (J * R);
	protected static final double b = k_t * G /(J * R);

	private static final double kSimTime = 0.00001;
	private static final double h = 0.0001;
	
	private static final double maxAngle = Math.PI / 2.0; //When arm is  all the way down
	private static final double minAngle = 0.0; //When the arm is all the way up
	
	private double ang, vel;
	
	private static final ArmSim instance = new ArmSim();
	
	private ArmSim() {
		ang = 0.0;
		vel = 0.0;
	}
	
	public static ArmSim getInstance() {
		return instance;
	}
	
	public void reset() {
		ang = 0.0;
		vel = 0.0;
	}
	
	public double getAccel(double voltage) {
		//System.out.println("angle: " + r2d(ang) + " velocity: " + r2d(vel));
		return a * vel + b * voltage; 
	}
	
	public void simulateTime(double voltage, double dt) {
		double currTime = 0.0;
		while(currTime < dt) {
			ang += vel * kSimTime;
			vel += getAccel(voltage) * kSimTime;
			currTime += kSimTime;
		}
	}
	
	public void RK4(double voltage, double dt) {
		double currTime = 0.0;
		while(currTime < dt) {
			double currVel = vel;
			double currAng = ang;
			double[] k1 = {vel, getAccel(voltage)};
			ang = currAng + h / 2.0 * k1[0];
			vel = currVel + h / 2.0 * k1[1];
			double[] k2 = {vel, getAccel(voltage)};
			ang = currAng + h / 2.0 * k2[0];
			vel = currVel + h / 2.0 * k2[1];
			double[] k3 = {vel, getAccel(voltage)};
			ang = currAng + h * k3[0];
			vel = currVel + h * k3[1];
			double[] k4 = {vel, getAccel(voltage)};
			ang = currAng + h / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]);
			vel = currVel + h / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]);
			currTime += h;
		}
	}
	
	public double getAngle() {
		return r2d(ang);
	}
	
	public double getVel() {
		return vel;
	}
	
	public boolean violatedLimits() {
		if(ang > maxAngle || ang < minAngle) {
			return true;
		} else {
			return false;
		}
	}

	public static double r2d(double radians) {
		return radians * 180.0 / Math.PI;
	}
	
	public static double d2r(double degrees) {
		return degrees * Math.PI / 180.0;
	}
}
