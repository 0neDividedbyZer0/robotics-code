package org.usfirst.frc.team1671.auto.modes;

import org.usfirst.frc.team1671.auto.AutoModeBase;
import org.usfirst.frc.team1671.auto.AutoModeEndedException;

public class DoNothingMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		System.out.println("Doing nothing");
	}
	
}
