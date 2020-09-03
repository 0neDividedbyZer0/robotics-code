package frc.statemachines;

import frc.robot.Constants;

/**
 * 
 * @author Maverick Zhang
 * All information about the current superstructure is known here. That can be found in IntakeState.
 *
 */
public class SuperstructureState {
	public double height = Constants.ELEVATOR_MIN;
	public boolean hasPanel = false;
	
	public SuperstructureState(double height, boolean hasPanel) {
		this.height = height;
		this.hasPanel = hasPanel;
	}
	
	public SuperstructureState() {
		this(Constants.ELEVATOR_MIN, false);
	}
	
	public SuperstructureState(double height) {
		this(height, false);
	}
	
	public SuperstructureState(SuperstructureState other) {
		this.height = other.height;
		this.hasPanel = other.hasPanel;
	}
	


}
