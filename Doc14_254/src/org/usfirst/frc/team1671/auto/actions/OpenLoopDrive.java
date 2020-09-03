package org.usfirst.frc.team1671.auto.actions;

import org.usfirst.frc.team1671.robot.DriveSignal;
import org.usfirst.frc.team1671.subsystems.Drive;
import org.usfirst.frc.team1671.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Timer;

public class OpenLoopDrive implements Action {
	private static final Drive drive = Drive.getInstance();
	private static final Superstructure superstructure = Superstructure.getInstance();
	
	private double left,right,duration;
	private boolean untilSeeCube;
	private double startTime;

	
	
	public OpenLoopDrive(double left, double right, double duration, boolean untilSeeCube) {
		this.left = left;
		this.right = right;
		this.duration = duration;
		this.untilSeeCube = untilSeeCube;
	}
	
	@Override
	public void start() {
		drive.setOpenLoop(new DriveSignal(left,right));
		startTime = Timer.getFPGATimestamp();
	}
	
	@Override
	public void update() {
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() - startTime > duration || (untilSeeCube && superstructure.getObservedState().hasCube);
	}

	@Override
	public void done() {
		drive.setOpenLoop(DriveSignal.BRAKE);
	}

	

}
