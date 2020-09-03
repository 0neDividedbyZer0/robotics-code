package org.usfirst.frc.team1671.auto.actions;

import java.util.List;

import org.usfirst.frc.team1671.subsystems.Drive;
import org.usfirst.frc.team1671.subsystems.Drive.DriveState;

import trajectory_lib.AutoTrajectory;
import trajectory_lib.Trajectory;
import trajectory_lib.TrajectoryGenerator;
import util.PathfinderGenerator;

public class FollowPathSaved implements Action {
	private static final Drive drive = Drive.getInstance();
	private static AutoTrajectory auto;
	
	private final boolean reverse;
	
	public FollowPathSaved(String path, boolean reverse) {
		this.reverse = reverse;
		auto = TrajectoryGenerator.loadAutoTrajectory(path);
	}
	
	@Override
	public void start() {
		drive.setTrajectory(auto, reverse);
		drive.setState(DriveState.PATH_FOLLOWING);
		drive.setBrakeMode(true);
	}
	
	@Override
	public void update() {
	}

	@Override
	public boolean isFinished() {
		return drive.pathFinished();
	}

	@Override
	public void done() {
		drive.stop();
	}

	

}