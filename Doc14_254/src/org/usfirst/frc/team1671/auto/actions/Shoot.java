package org.usfirst.frc.team1671.auto.actions;

import org.usfirst.frc.team1671.states.IntakeState;
import org.usfirst.frc.team1671.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;

public class Shoot implements Action {
	private static final Intake intake = Intake.getInstance();
	private static final double shootTime = 0.35;
	
	private final double power;
	private double startTime;
	private IntakeState wanted;
	
	public Shoot(double power) {
		this.power = power;
	}
	
	@Override
	public void start() {
		startTime = Timer.getFPGATimestamp();
		wanted = intake.getWantedState();
		wanted.setPower(power);
		
		if(wanted.jawState != IntakeState.JawState.NEUTRAL) {
			wanted.jawState = IntakeState.JawState.NEUTRAL;
		}
		intake.setWantedState(wanted);
	}
	
	@Override
	public void update() {
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() - startTime > shootTime;
	}

	@Override
	public void done() {
		wanted = intake.getWantedState();
		wanted.setPower(0.0);
		wanted.jawState = IntakeState.JawState.NEUTRAL;
		intake.setWantedState(wanted);
	}

	

}
