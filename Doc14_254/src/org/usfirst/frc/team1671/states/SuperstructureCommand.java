package org.usfirst.frc.team1671.states;

import org.usfirst.frc.team1671.robot.RobotMap;

/**
 * 
 * @author Maverick Zhang
 * Tells the Superstructure what voltage to run the elevator, and whether or not you want manual control.
 *
 */
public class SuperstructureCommand {
	public double height = RobotMap.MIN_ELEV_HEIGHT;
	public double angle = RobotMap.ARM_MIN_ANGLE;
	
	public boolean openLoopElevator = false;
	public double openLoopElevatorPercent = 0.0;
}
