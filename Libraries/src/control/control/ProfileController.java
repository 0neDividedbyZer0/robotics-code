package control;

import edu.wpi.first.wpilibj.util.BoundaryException;

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
	private boolean off = true;
	
	public ProfileController(double kP, double kI, double kD, double kV) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kV = kV;
	}
	
	public ProfileController() {
		
	}
	
	public ProfileController(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kV = 0.0;
	}
	
	//measurement should just be position
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
		
		prevError = error;
		
		if(result > maxOutput) {
			result = maxOutput;
		} else if(result < minOutput) {
			result = minOutput;
		}
		
		return result;
	}
	
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
			throw new BoundaryException("Min is larger than max.");
		}
		this.minInput = minInput;
		this.maxInput = maxInput;
	}
	
	public void setOutputRange(double minOutput, double maxOutput) {
		if(minOutput > maxOutput) {
			throw new BoundaryException("Min is larger than max.");
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
