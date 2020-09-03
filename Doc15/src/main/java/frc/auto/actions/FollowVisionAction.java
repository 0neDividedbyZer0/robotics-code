package frc.auto.actions;

import java.util.Arrays;
import java.util.List;

import frc.robot.Constants;
import frc.subsystems.Drive;
import frc.subsystems.LimelightProcessor;
import frc.util.DriveSignal;
import trajectory_lib.AutoTrajectory;
import trajectory_lib.Trajectory;
import trajectory_lib.TrajectoryGenerator;
import trajectory_lib.Waypoint;

public class FollowVisionAction implements Action {
    private Drive drive = Drive.getInstance();
    private LimelightProcessor processor  = LimelightProcessor.getInstance();
    private boolean valid;

    public FollowVisionAction() {
        double tv = processor.getTV();
        if(tv == 1.0) {
            valid = true;
            double directDistance = 0.9 * Math.hypot( processor.getX(), processor.getZ());
            double delta_x = 1.0 * directDistance / 12.0 * Math.cos(Math.toRadians(drive.getHeading() - processor.getTX()));//-0.75 * processor.getZ()/ 12.0;//0.9 / 12.0 * processor.getDist() * Math.cos(Math.toRadians(drive.getHeading() - processor.getTX()));
            double delta_y = 1.0 * directDistance / 12.0 * Math.sin(Math.toRadians(drive.getHeading() - processor.getTX()));//0.64 *processor.getX()/ 12.0;//2.3 / 12.0 * processor.getDist() * Math.sin(Math.toRadians(drive.getHeading() - processor.getTX()));
            double delta_theta = Math.toRadians(drive.getHeading() - processor.getTX());// - processor.getYaw());
            //TODO: check if ts needs to be + or -
            List<Waypoint> path = Arrays.asList(new Waypoint(0.0, 0.0, drive.getHeading()), new Waypoint(delta_x, delta_y, 1.0 * delta_theta));// - processor.getTS())));
            Trajectory trajectory = TrajectoryGenerator.generateQuinticHermiteSpline(drive.getTestConfig(), path);
            AutoTrajectory traj = TrajectoryGenerator.makeLeftRightTrajectories(trajectory, Constants.WHEEL_BASE);
            drive.setTrajectory(traj, false);
        } else {
            valid = false;
        }
        
    }

    @Override
    public void start() {
        if(valid) {
            drive.startPathFollowing();
        }
        
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        if(valid) {
            return drive.pathIsFinished();
        } else {
            return true;
        }
        
    }

    @Override
    public void done() {
        drive.setOpenLoop(DriveSignal.NEUTRAL);
    }

}