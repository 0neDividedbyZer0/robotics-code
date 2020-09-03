package org.usfirst.frc.team1671.subsystems;

import org.usfirst.frc.team1671.loops.Loop;
import org.usfirst.frc.team1671.loops.Looper;
import org.usfirst.frc.team1671.robot.RobotMap;
import org.usfirst.frc.team1671.statemachines.SuperstructureStateMachine;
import org.usfirst.frc.team1671.statemachines.SuperstructureStateMachine.SystemState;
import org.usfirst.frc.team1671.states.IntakeState;
import org.usfirst.frc.team1671.states.SuperstructureCommand;
import org.usfirst.frc.team1671.states.SuperstructureState;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class Superstructure extends Subsystem {
	
	static Superstructure instance = null;
	
	public static Superstructure getInstance() {
		if(instance == null) {
			instance = new Superstructure();
		}
		return instance;
	}
	
	private Intake intake = Intake.getInstance();
	private Arm arm = Arm.getInstance();
	private Elevator elevator = Elevator.getInstance();
	private SuperstructureState state = new SuperstructureState();
	private SuperstructureStateMachine stateMachine = new SuperstructureStateMachine();
	private SuperstructureStateMachine.WantedState wantedState = SuperstructureStateMachine.WantedState.IDLE; 
	
	private Loop loop = new Loop() {
		@Override
		public void onStart(double timestamp) {
			stateMachine.resetManual();
		}
		
		@Override
		public void onLoop(double timestamp) {
			synchronized(Superstructure.this) {
				updateObservedState(state);
				
				SuperstructureCommand command = stateMachine.update(timestamp, wantedState, state);
				setFromCommand(command);
			}
		}
		
		@Override
		public void onStop(double timestamp) {
			
		}
	};
	
	public synchronized SuperstructureStateMachine.SystemState getSuperstructureState() {
		return stateMachine.getSystemState();
	}
	
	public synchronized SuperstructureState getObservedState() {
		return state;
	}
	
	private synchronized void updateObservedState(SuperstructureState state) {
		state.height = elevator.getHeight();
		state.angle = arm.getAngle();
		state.intakeClamped = intake.getJawState() == IntakeState.JawState.NEUTRAL;
		state.hasCube = intake.getCurrentState().cubeDetected;
	}
	
	//This is the method that actually does stuff!!!!!!
	synchronized void setFromCommand(SuperstructureCommand command) {
		if(command.openLoopElevator) {
			elevator.setOpenLoop(command.openLoopElevatorPercent);
			
		} else {
			elevator.setProfiled();
			elevator.setGoal(command.height);
		}
		
		arm.setProfiled();
		arm.setGoal(command.angle);
	}
	
	private double applySafety(double rawPower) {
		if(rawPower > 0.0 && state.height <= RobotMap.MANUAL_MIN_HEIGHT) {
			return 0.0;
		} else if(rawPower < 0.0 && state.height >= RobotMap.MANUAL_MAX_HEIGHT) {
			return 0.0;
		} else {
			return rawPower;
		}
	}
	
	public synchronized void setOpenLoopPower(double power) {
		stateMachine.setOpenLoopPower(applySafety(power));
		wantedState = SuperstructureStateMachine.WantedState.MANUAL;
	}
	
	@Override
	public void outputToSmartDashboard() {
	}

	@Override
	public void stop() {
	}

	@Override
	public void zeroSensors() {
	}

	@Override
	public void registerEnabledLoops(Looper enabledLooper) {
		enabledLooper.register(loop);
	}
	
	public synchronized void setDesiredAngle(double angle) {
		stateMachine.setWantedAngle(angle);
		///wantedState = SuperstructureStateMachine.WantedState.MOVE_TO_POSITION;
	}
	
	public synchronized void setDesiredHeight(double height) {
		stateMachine.setWantedHeight(height);
		wantedState = SuperstructureStateMachine.WantedState.MOVE_TO_POSITION;
	}

}
