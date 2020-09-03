package org.usfirst.frc.team1671.auto.modes;

import org.usfirst.frc.team1671.auto.AutoModeBase;
import org.usfirst.frc.team1671.auto.AutoModeEndedException;
import org.usfirst.frc.team1671.auto.actions.SimpleDrive;

public class SwitchMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new SimpleDrive(4.5));
	}
	

}
