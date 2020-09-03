package frc.statemachines;

import frc.robot.Constants;

/**
 * 
 * @author Maverick Zhang
 * Tells the Superstructure what voltage to run the elevator, and whether or not you want manual control.
 *
 */
public class SuperstructureCommand {
	public double height = Constants.ELEVATOR_MIN;
	
	public boolean openLoopElevator = false;
	public double openLoopElevatorPercent = 0.0;
}
