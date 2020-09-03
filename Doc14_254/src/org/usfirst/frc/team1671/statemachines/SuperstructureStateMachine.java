package org.usfirst.frc.team1671.statemachines;

import org.usfirst.frc.team1671.robot.RobotMap;
import org.usfirst.frc.team1671.states.SuperstructureCommand;
import org.usfirst.frc.team1671.states.SuperstructureState;

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
	private double maxHeight = RobotMap.MAX_ELEV_HEIGHT;
	private double minAngle = RobotMap.ARM_MIN_ANGLE;
	
	public synchronized SystemState getSystemState() {
		return systemState;
	}
	
	public synchronized void setOpenLoopPower(double power) {
		openLoopPower = power;
	}
	
	public synchronized void setMaxHeight(double maxHeight) {
		this.maxHeight = maxHeight;
	}
	
	public synchronized void setMinAngle(double minAngle) {
		this.minAngle = minAngle;
	}
	
	public synchronized void setWantedAngle(double wantedAngle) {
		commandedState.angle = wantedAngle;
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
			
			if(current.hasCube) {
				setMinAngle(RobotMap.STOW_ANGLE_CUBE);
			} else {
				setMinAngle(RobotMap.ARM_MIN_ANGLE);
			}
			
			//NOTE: THIS MIGHT NEED TO BE CHANGED TO current.angle < 45.0 && current.hasCube
			if(commandedState.angle < 45.0 && current.hasCube) {
				setMaxHeight(RobotMap.SAFE_CUBE_HEIGHT);
			} else {
				setMaxHeight(RobotMap.MAX_ELEV_HEIGHT);
			}
			
			//suspected cause of arm issue, moving command.angle = Math.max(commandedState.angle, minAngle); will fix
			if(!command.openLoopElevator) {
				command.height = Math.min(commandedState.height, maxHeight);
				command.angle = Math.max(commandedState.angle, minAngle);
			}
			
			
			
			switch(systemState) {
				case OPEN_LOOP:
					getManualCommandedState();
					break;
				case AUTO:
					getAutoCommandedState();
					break;
				default:
					System.out.println("Unexpected superstructure state output state: " + systemState);
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
