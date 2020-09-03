package HolonomicDrive;

public class HolonomicDrive {
	private static double maxOutput = 1.0;
	private static double deadzone = 0.02;
	
	public HolonomicDrive() {
	}
	
	private double clamp(double value) {
		double sign = Math.signum(value);
		if(Math.abs(value) > maxOutput) {
			return sign * maxOutput;
		} else {
			return value;
		}
	}
	
	private double squareValue(double value) {
		double sign = Math.signum(value);
		return value * value * sign;
	}
	
	private double deadzone(double value) {
		if(Math.abs(value) < 0.02) {
			return 0.0;
		} else {
			double scaledValue;
			if(value < 0.0) {
				scaledValue = (value + deadzone) / (-maxOutput + deadzone);
			} else {
				scaledValue = (value - deadzone) / (maxOutput - deadzone);
			}
			return scaledValue;
		}
	}
	
	private void normalize(double[] speeds) {
		double max = Math.abs(speeds[0]);
		for(int i = 1; i < speeds.length; i++) {
			double temp = Math.abs(speeds[i]);
			if(max < temp) {
				max = temp;
			}
		}
		if(max > 1.0) {
			for(int i = 0; i < speeds.length; i++) {
				speeds[i] = speeds[i] / max;
			}
		}
	}
	
	//Every wheel should go to the left with positive values.
	/*
	 * You should make drive the leftStickY, strafe the leftStickX, and rotation the rightStickX
	 * 
	 * Incomplete
	 */
	public double[] holonomicDrive(double drive, double strafe, double rotation, boolean squaredInputs, boolean diagonal) {
		double y_speed, x_speed, rot_speed;
		
		if(diagonal) {
			if(squaredInputs) {
				y_speed = clamp(squareValue(drive));
				x_speed = clamp(squareValue(strafe));
				rot_speed = clamp(squareValue(rotation));
			} else {
				y_speed = clamp(drive);
				x_speed = clamp(strafe);
				rot_speed = clamp(rotation);
			}
			
			y_speed = deadzone(drive);
			x_speed = deadzone(strafe);
			rot_speed = deadzone(rot_speed);
			
			double tr_wheel = x_speed - y_speed - rot_speed;
			double tl_wheel = x_speed + y_speed - rot_speed;
			double dl_wheel = -x_speed + y_speed - rot_speed;
			double dr_wheel = -x_speed -y_speed - rot_speed;
			
			double[] speeds = new double[] {tr_wheel, tl_wheel, dl_wheel, dr_wheel};
			normalize(speeds);
			return speeds;	
		} else {
			if(squaredInputs) {
				y_speed = clamp(squareValue(drive));
				x_speed = clamp(squareValue(strafe));
				rot_speed = clamp(squareValue(rotation));
			} else {
				y_speed = clamp(drive);
				x_speed = clamp(strafe);
				rot_speed = clamp(rotation);
			}
			
			y_speed = deadzone(drive);
			x_speed = deadzone(strafe);
			rot_speed = deadzone(rot_speed);
			
			double t_wheel = x_speed + rot_speed;
			double d_wheel = -x_speed + rot_speed;
			double l_wheel = y_speed + rot_speed;
			double r_wheel = -y_speed + rot_speed;
			
			double[] speeds = new double[] {l_wheel, t_wheel, r_wheel, d_wheel};
			normalize(speeds);
			return speeds;	
		}
	}
	
	public double[] holonomicDrive(double drive, double strafe, double rotation) {
		return holonomicDrive(drive, strafe, rotation, true, true);
	}

}
