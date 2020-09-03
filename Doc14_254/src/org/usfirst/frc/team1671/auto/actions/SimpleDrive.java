package org.usfirst.frc.team1671.auto.actions;

import org.usfirst.frc.team1671.robot.DriveSignal;
import org.usfirst.frc.team1671.subsystems.Drive;
import org.usfirst.frc.team1671.subsystems.Drive.DriveState;

import edu.wpi.first.wpilibj.Timer;
import util.Util;

//Drive with 1d profile
public class SimpleDrive implements Action {
	private static final Drive drive = Drive.getInstance();
	private static final double epsilon = 2.0;
	private static final double timeout = 7.0;
	
	private double distance;
	private final boolean reverse;
	private double startTime;
	
	public SimpleDrive(double distance, boolean reverse) {
		this.distance = distance;
		this.reverse = reverse;
	}
	
	public SimpleDrive(double distance) {
		this(distance, false);
	}
	
	@Override
	public void start() {
		startTime = Timer.getFPGATimestamp();
		if(reverse) {
			distance *= -1.0;
		}
		drive.setSimpleGoal(distance);
		drive.setState(DriveState.SIMPLE_DRIVE);
	}
	
	@Override
	public void update() {
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() - startTime > timeout || Util.epsilonEquals(drive.getRightEnc(), drive.getSimpleGoal(), epsilon);
	}

	@Override
	public void done() {
		drive.setOpenLoop(DriveSignal.BRAKE);
	}

	
	
	
	

}
