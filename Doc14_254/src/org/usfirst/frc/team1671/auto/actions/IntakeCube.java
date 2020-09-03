package org.usfirst.frc.team1671.auto.actions;

import org.usfirst.frc.team1671.robot.RobotMap;
import org.usfirst.frc.team1671.states.IntakeState;
import org.usfirst.frc.team1671.states.SuperstructureState;
import org.usfirst.frc.team1671.subsystems.Intake;
import org.usfirst.frc.team1671.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Timer;
import util.Util;

public class IntakeCube implements Action {
	private static final Superstructure superstructure = Superstructure.getInstance();
	private static final Intake intake = Intake.getInstance();
	private static final double heightEpsilon = 2.0;
	private static final double angleEpsilon = 5.0;
	private static final double maxTime = 1.0;
	
	private final boolean waitUntilCube, move;
	private double startTime;
	private IntakeState wanted;
	
	public IntakeCube(boolean move, boolean waitUntilCube) {
		this.move = move;
		this.waitUntilCube = waitUntilCube;
		wanted = intake.getWantedState();
	}
	
	@Override
	public void start() {
		if(move) {
			superstructure.setDesiredAngle(101.0);
			superstructure.setDesiredHeight(RobotMap.MIN_ELEV_HEIGHT);
		}
		startTime = Timer.getFPGATimestamp();
		if(!intake.getCurrentState().cubeDetected) {
			wanted.setPower(0.75);
			intake.setWantedState(wanted);
		}
	}
	
	@Override
	public void update() {
		SuperstructureState state = superstructure.getObservedState();
		if(move && (Util.epsilonEquals(state.height, RobotMap.MIN_ELEV_HEIGHT, heightEpsilon) 
				&& Util.epsilonEquals(state.angle, RobotMap.ARM_MIN_ANGLE, angleEpsilon))) {
			wanted.setPower(0.75);
			intake.setWantedState(wanted);
		}
	}

	@Override
	public boolean isFinished() {
		if(waitUntilCube) {
			boolean timeout = Timer.getFPGATimestamp() - startTime > maxTime;
			if(timeout) {
				System.out.println("Intake timed out");
				return true;
			}
			return intake.getCurrentState().cubeDetected;
		} else {
			return true; 
		}
	}

	@Override
	public void done() {
		wanted = new IntakeState();
		wanted.jawState = IntakeState.JawState.CLOSED;
		wanted.setPower(0.1);
		intake.setWantedState(wanted);
	}

	

}
