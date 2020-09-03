package org.usfirst.frc.team1671.auto.actions;

import java.util.List;

import org.usfirst.frc.team1671.robot.RobotMap;
import org.usfirst.frc.team1671.subsystems.Drive;
import org.usfirst.frc.team1671.subsystems.Drive.DriveState;

import trajectory_lib.AutoTrajectory;
import trajectory_lib.Trajectory;
import trajectory_lib.TrajectoryGenerator;
import trajectory_lib.Waypoint;
import util.PathfinderGenerator;

public class FollowPath implements Action {
	private static final Drive drive = Drive.getInstance();
	
	private final List<Waypoint> points;
	private final boolean reverse;
	private final Trajectory traj;
	private final AutoTrajectory auto;
	
	public FollowPath(List<Waypoint> waypoints, boolean reverse) {
		points = waypoints;
		this.reverse = reverse;
		traj = TrajectoryGenerator.generate(RobotMap.config, waypoints);
		auto = TrajectoryGenerator.makeLeftRightTrajectories(traj, RobotMap.WHEELBASE);
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
