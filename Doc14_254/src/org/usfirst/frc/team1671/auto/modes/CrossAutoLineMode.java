package org.usfirst.frc.team1671.auto.modes;

import org.usfirst.frc.team1671.auto.AutoModeBase;
import org.usfirst.frc.team1671.auto.AutoModeEndedException;
import org.usfirst.frc.team1671.auto.actions.OpenLoopDrive;
import org.usfirst.frc.team1671.auto.actions.Wait;

public class CrossAutoLineMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		System.out.println("Running Cross Auto Mode");
		runAction(new Wait(5.0));
		runAction(new OpenLoopDrive(0.3, 0.3, 3.5, false));
	}

}
