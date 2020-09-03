package frc.statemachines;

import control.ProfileConstraints;
import control.ProfileController;
import control.ProfilePoint;
import control.TrapezoidalProfile;
import frc.robot.Constants;
import frc.statemachines.SuperstructureCommand;
import frc.statemachines.SuperstructureState;

/**
 * 
 * @author Maverick Zhang
 * A finite state machine class for the superstructure. It is designed to handle transitions between states of the
 * superstructure.
 *
 */
public class SuperstructureStateMachine {
	
	public enum SystemState {
		OPEN_LOOP,
		AUTO
	}
	
	public enum WantedState {
		IDLE,
		MOVE_TO_POSITION,
		MANUAL
	}
	
	private SystemState systemState = SystemState.AUTO;
	
	
	//These are variables that hold the current command and state of the superstructure.
	private SuperstructureCommand command = new SuperstructureCommand();
	private SuperstructureState commandedState = new SuperstructureState();
	
	//This is required for manual elevator controol
	private double openLoopPower = 0.0;
	private double maxHeight = Constants.ELEVATOR_MAX;
	private double minHeight = Constants.ELEVATOR_MIN;

	
	
	public synchronized SystemState getSystemState() {
		return systemState;
	}
	
	public synchronized void setOpenLoopPower(double power) {
		openLoopPower = power;
	}
	
	public synchronized void setWantedHeight(double wantedHeight) {
		commandedState.height = wantedHeight;
	}
	
	/**
	 * This is the heart of the class. This will try to make the superstructure state
	 * approach the wanted state
	 * @param timestamp When this was run (just use Timer.getFPGATimestamp())
	 * @param wanted Your desired state
	 * @param current Your current state
	 * @return A command that should be fed in to the superstructure to cause the wanted and current states to converge.
	 */
	public synchronized SuperstructureCommand update(double timestamp, WantedState wanted, SuperstructureState current) {
		synchronized(SuperstructureStateMachine.this) {
			SystemState newState;
			
			switch(systemState) {
				case OPEN_LOOP:
					newState = handleManualTransitions(wanted, current);
					break;
				case AUTO:
					newState = handleAutoTransitions(wanted, current);
					break;
				default:
					System.out.println("Unexpected superstructure state: " + systemState);
					newState = systemState;
			}
			
			if(newState != systemState) {
				System.out.println(timestamp + ": Superstructure changed state: " + systemState + " -> " + newState);
				systemState = newState;
			}
			
			//Plz, don't make elevator_max smaller than elevator_min
			if(commandedState.height > Constants.ELEVATOR_MAX) {
				command.height = Constants.ELEVATOR_MAX;
			} else if(commandedState.height < Constants.ELEVATOR_MIN) {
				command.height = Constants.ELEVATOR_MIN;
			} else {
				command.height = commandedState.height;
			}
			
			switch(systemState) {
				case OPEN_LOOP:
					getManualCommandedState();
					break;
				case AUTO:
					getAutoCommandedState();
					break;
				default:
					System.out.println("Unexpected superstructure command: " + systemState);
                    break;
			}
			
			return command;
		}
		
	}
	
	private SystemState handleDefaultTransitions(WantedState wanted, SuperstructureState state) {
		if(wanted == WantedState.MOVE_TO_POSITION) {
			return SystemState.AUTO;	
		} else if(wanted == WantedState.MANUAL) {
			return SystemState.OPEN_LOOP;
		} else {
			return SystemState.AUTO;
		}
	}
	
	private void getAutoCommandedState() {
		command.openLoopElevator = false;
		command.openLoopElevatorPercent = 0.0;

		
	}
	
	private SystemState handleAutoTransitions(WantedState wanted, SuperstructureState current) {
		return handleDefaultTransitions(wanted, current);
	}
	
	public synchronized void resetManual() {
		openLoopPower = 0.0;
	}
	
	private void getManualCommandedState() {
		command.openLoopElevator = true;
		command.openLoopElevatorPercent = openLoopPower;
	}
	
	private SystemState handleManualTransitions(WantedState wanted, SuperstructureState current) {
		if(wanted != WantedState.MANUAL) {
			return handleDefaultTransitions(WantedState.MOVE_TO_POSITION, current);
		}
		return handleDefaultTransitions(wanted, current);
	}
}
