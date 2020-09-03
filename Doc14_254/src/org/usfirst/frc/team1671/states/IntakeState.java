package org.usfirst.frc.team1671.states;

/**
 * 
 * @author Maverick Zhang
 * Auxiliary class for managing the intake. Includes voltage, power cube detection, and jaw state
 */
public class IntakeState {
	public enum JawState {
		OPEN,
		NEUTRAL,
		CLOSED
	}
	
	public JawState jawState = JawState.NEUTRAL;
	public double leftMotor = 0.;
	public double rightMotor = 0.;
	
	public boolean cubeDetected = false;
	
	public void setPower(double power) {
		leftMotor = rightMotor = power;
	}
	
	public boolean detectedCube() {
		return cubeDetected;
	}

}
