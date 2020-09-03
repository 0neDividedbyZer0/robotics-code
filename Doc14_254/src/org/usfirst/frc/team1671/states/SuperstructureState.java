package org.usfirst.frc.team1671.states;

import org.usfirst.frc.team1671.robot.RobotMap;

/**
 * 
 * @author Maverick Zhang
 * All information about the current superstructure is known here. That can be found in IntakeState.
 *
 */
public class SuperstructureState {
	public double height = RobotMap.MIN_ELEV_HEIGHT;
	public double angle = RobotMap.ARM_MIN_ANGLE;
	public boolean intakeClamped = true;
	
	public boolean hasCube = false;
	
	public SuperstructureState(double height, double angle, boolean intakeClamped) {
		this.height = height;
		this.angle = angle;
		this.intakeClamped = intakeClamped;
	}
	
	public SuperstructureState() {
		this(RobotMap.MIN_ELEV_HEIGHT, RobotMap.ARM_MIN_ANGLE, true);
	}
	
	public SuperstructureState(double height, double angle) {
		this(height, angle, true);
	}
	
	public SuperstructureState(SuperstructureState other) {
		this.height = other.height;
		this.angle = other.angle;
		this.intakeClamped = other.intakeClamped;
	}
	


}
