package frc.subsystems;

import frc.loops.Loop;
import frc.loops.Looper;
import frc.robot.Constants;
import frc.statemachines.SuperstructureCommand;
import frc.statemachines.SuperstructureState;
import frc.statemachines.SuperstructureStateMachine;
import frc.statemachines.SuperstructureStateMachine.SystemState;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {
	
	private static Superstructure instance = null;
	
	public static Superstructure getInstance() {
		if(instance == null) {
			instance = new Superstructure();
		}
		return instance;
	}

	private final double tolerance = 0.5; //inches
	

	private Solenoid molly = new Solenoid(Constants.MOLLY_SOL);

	private PowerDistributionPanel pdp = new PowerDistributionPanel();
	
	private Intake intake = Intake.getInstance();
	private Elevator elevator = Elevator.getInstance();
	private SuperstructureState state = new SuperstructureState();
	private SuperstructureStateMachine stateMachine = new SuperstructureStateMachine();
	private SuperstructureStateMachine.WantedState wantedState = SuperstructureStateMachine.WantedState.IDLE; 

	private double lowerIntakeStartTime;

	public Superstructure() {
		molly.set(true);
	}
	
	
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


				/*boolean intakeSafe = false;
				if(command.height != state.height) {
					lowerIntakeStartTime = timestamp;
					if(intake.intakeValue() != Value.kReverse) {
						intake.getWantedState().extend = false;
						intakeSafe = false;
					} else {
						intakeSafe = true;
					}
				}

				if(!intakeSafe) {
					intakeSafe = timestamp - lowerIntakeStartTime > 0.5;
				}*/
				
				

				
				
				setFromCommand(command, true);
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
		state.hasPanel = intake.getBanner();
		state.height = elevator.getLeftEncoder();
	}
	
	//This is the method that actually does stuff!!!!!!
	private synchronized void setFromCommand(SuperstructureCommand inputCommand, boolean safe) {
		if(inputCommand.openLoopElevator) {
			elevator.setOpenLoop(applySafety(inputCommand.openLoopElevatorPercent));
			
		} else {
			if(safe) {
				elevator.setGoal(inputCommand.height);
			}
		}
		
		
		
	}

	private synchronized double applySafety(double signal) {
		double safeSignal = signal;
		if(Math.abs(state.height - Constants.ELEVATOR_MAX) < tolerance) {
			if(signal > 0.0) {
				safeSignal = 0.0;
			} 
		} else if(Math.abs(state.height - Constants.ELEVATOR_MIN) < tolerance) {
			if(signal < 0.0) {
				safeSignal = 0.0;
			} 
		}
		return safeSignal;
	}
	
	
	public synchronized void setOpenLoopPower(double power) {
		stateMachine.setOpenLoopPower(power);
		wantedState = SuperstructureStateMachine.WantedState.MANUAL;
	}
	
	@Override
	public void outputToSmartDashboard() {
		/*SmartDashboard.putNumber("Left Front Drive Current", pdp.getCurrent(14));
		SmartDashboard.putNumber("Left Rear Drive Current", pdp.getCurrent(15));
		SmartDashboard.putNumber("Right Front Drive Current", pdp.getCurrent(0));
		SmartDashboard.putNumber("Right Rear Drive Current", pdp.getCurrent(1));*/
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
	
	
	public synchronized void setDesiredHeight(double height) {
		stateMachine.setWantedHeight(height);
		stateMachine.setOpenLoopPower(0.0);
		wantedState = SuperstructureStateMachine.WantedState.MOVE_TO_POSITION;
	}

}
