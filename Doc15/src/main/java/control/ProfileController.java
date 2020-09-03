package control;




/**
 * 
 * @author Maverick Zhang
 * Based on 254's synchronous PID controller. This controller essentially is PID, and has support
 * for continuous inputs, prevention of integral windup, and motion profiles. (kV is a feedforward constant 
 * that only works for motion profiling). This is designed for subsystems, EXCEPT for Drive
 */
public class ProfileController {
	private double kP, kI, kD, kV;
	private double maxOutput = 1.0;
	private double minOutput = -1.0;
	private double maxInput = 0.0;
	private double minInput = 0.0;
	private boolean continuousInput = false; //For sensors that wrap around e.g. absolute encoder since values go from 0 to 359
	private double prevError = 0.0;
	private double totalError = 0.0;
	private double setpoint = 0.0;
	private double error = 0.0;
	private double result = 0.0;
	private double lastInput = Double.NaN;
	private double deadband = 0.0;
	
	public ProfileController(double kP, double kI, double kD, double kV) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kV = kV;
	}
	
	public ProfileController() {
		
	}
	
	public ProfileController(double kP, double kI, double kD) {
		this(kP, kI, kD, 0.0);
	}
	
	//measurement should just be position
	/**
	 * This is one of the controller's two calculate functions. This one 
	 * is designed for usage with the TrapezoidalProfile class.
	 * @param goal The current setpoint given from the Trapezoidal Profile
	 * @param measurement The current sensor measurement
	 * @param dt The speed of the loop
	 * @return
	 */
	public double calculate(ProfilePoint goal, double measurement, double dt) {
		if(dt < 1.0E-6) {
			dt = 1.0E-6;
		}
		lastInput = measurement;
		setpoint = goal.getPos();
		error =  setpoint - measurement;
		if(continuousInput) {
			if(Math.abs(error) > (maxInput - minInput) / 2.0) {
				if(error > 0.0) {
					error  = error - maxInput + minInput;
				} else {
					error  = error + maxInput - minInput;
				}
			}
		}
		
		if((error * kP < maxOutput) && (error * kP > minOutput)) {
			totalError += error * dt;
		} else {
			totalError = 0.0;
		}
		
		double propError;
		if(Math.abs(error) < deadband) {
			propError = 0.0;
		} else {
			propError = error;
		}
		
		result = kP * propError + kI * totalError + kD * (error - prevError) / dt + kV * goal.getVel();
		//System.out.println(result);
		prevError = error;
		
		if(result > maxOutput) {
			result = maxOutput;
		} else if(result < minOutput) {
			result = minOutput;
		}
		
		return result;
	}
	
	/**
	 * One of the two calculate methods. This one is designed to only be
	 * used with a nonchanging setpoint. Does not have a feedforward
	 * @param measurement Current sensor reading
	 * @param dt Time of loop
	 * @return
	 */
	public double calculate(double measurement, double dt) {
		if(dt < 1.0E-6) {
			dt = 1.0E-6;
		}
		lastInput = measurement;
		error = setpoint - measurement;
		if(continuousInput) {
			if(Math.abs(error) > (maxInput - minInput) / 2.0) {
				if(error > 0.0) {
					error  = error - maxInput + minInput;
				} else {
					error  = error + maxInput - minInput;
				}
			}
		}
		
		if((error * kP < maxOutput) && (error * kP > minOutput)) {
			totalError += error * dt;
		} else {
			totalError = 0.0;
		}
		
		double propError;
		if(Math.abs(error) < deadband) {
			propError = 0.0;
		} else {
			propError = error;
		}
		
		result = kP * propError + kI * totalError + kD * (error - prevError) / dt;
		
		prevError = error;
		
		if(result > maxOutput) {
			result = maxOutput;
		} else if(result < minOutput) {
			result = minOutput;
		}
		
		return result;
	}
	
	public void setPID(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}
	
	public void setPID(double kP, double kI, double kD, double kV) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kV = kV;
	}
	
	public double getP() {
		return kP;
	}
	
	public double getI() {
		return kI;
	}
	
	public double getD() {
		return kD;
	}
	
	public double getV() {
		return kV;
	}
	
	public double get() {
		return result;
	}
	
	//When in continuous, min and max input are assumed to be the same point, so an if statement reduces the error as much
	//as possible
	public void setContinuous(boolean continuous) {
		continuousInput = continuous;
	}
	
	public void setDeadband(double deadband) {
		this.deadband = deadband;
	}
	
	public void setInputRange(double minInput, double maxInput) {
		if(minInput > maxInput) {
			System.out.println("Hey dummy, minInput is greater than maxInput");
		}
		this.minInput = minInput;
		this.maxInput = maxInput;
	}
	
	public void setOutputRange(double minOutput, double maxOutput) {
		if(minOutput > maxOutput) {
			System.out.println("Hey dummy, minOutput is greater than maxOutput");
		}
		this.minOutput = minOutput;
		this.maxOutput = maxOutput;
	}
	
	public void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
	}
	
	public double getSetpoint() {
		return setpoint;
	}
	
	public double getError() {
		return error;
	}
	
	public boolean onTarget(double tolerance) {
		return lastInput != Double.NaN && Math.abs(lastInput - setpoint) < tolerance;
	}
	
	public void reset() {
		lastInput = Double.NaN;
		prevError = 0.0;
		totalError = 0.0;
		result = 0.0;
		setpoint = 0.0;
	}
	
	public void resetIntegrator() {
		totalError = 0.0;
	}
	

	
	
	
	

}
