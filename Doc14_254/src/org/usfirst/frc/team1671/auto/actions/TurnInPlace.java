package org.usfirst.frc.team1671.auto.actions;

import org.usfirst.frc.team1671.robot.DriveSignal;
import org.usfirst.frc.team1671.subsystems.Drive;
import org.usfirst.frc.team1671.subsystems.Drive.DriveState;

import edu.wpi.first.wpilibj.Timer;
import util.Util;

public class TurnInPlace implements Action {
	private static final Drive drive = Drive.getInstance();
	private static final double epsilon = 1.0; //Degrees
	private static final double timeout = 2.0;
	
	private double angle;
	private boolean absolute;
	private double startTime;
	
	public TurnInPlace(double angle, boolean absolute) {
		this.angle = angle;
		this.absolute = absolute;
	}
	
	public TurnInPlace(double angle) {
		this(angle, false);
	}
	
	@Override
	public void start() {
		startTime = Timer.getFPGATimestamp();
		if(absolute) {
			drive.setAngleGoalAbs(angle);
		} else {
			drive.setAngleGoalRel(angle);
		}
		drive.setState(DriveState.TURN_TO_HEADING);
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() - startTime > timeout || Util.epsilonEquals(drive.getAngleGoal(), drive.getAngle(), epsilon);
	}

	@Override
	public void update() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void done() {
		drive.setOpenLoop(DriveSignal.BRAKE);
	}

	

}
