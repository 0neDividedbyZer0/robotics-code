package org.usfirst.frc.team1671.auto.actions;

import org.usfirst.frc.team1671.states.IntakeState;
import org.usfirst.frc.team1671.subsystems.Intake;

public class OpenCloseJaw implements Action {
	private static final Intake intake = Intake.getInstance();
	
	private final boolean open;
	
	public OpenCloseJaw(boolean open) {
		this.open = open;
	}
	
	@Override
	public void start() {
		IntakeState wanted = intake.getWantedState();
		if(open) {
			wanted.jawState = IntakeState.JawState.OPEN;
		} else {
			wanted.jawState = IntakeState.JawState.NEUTRAL;
		}
		intake.setWantedState(wanted);
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {
	}

	

}
