package frc.auto.modes;

import java.util.Arrays;
import java.util.List;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.EjectPanelAction;
import frc.auto.actions.FollowTrajectoryAction;
import frc.auto.actions.FollowVisionAction;
import frc.auto.actions.IntakeAction;
import frc.auto.actions.OpenLoopDriveAction;
import frc.auto.actions.SeriesAction;
import frc.auto.actions.TurnInPlaceAction;
import frc.auto.actions.Wait;
import frc.subsystems.Drive;
import frc.subsystems.LimelightProcessor;
import frc.util.FieldMeasurements;
import trajectory_lib.Waypoint;

public class TestDriveMode extends AutoModeBase {

    @Override
    public void routine() throws AutoModeEndedException {
        runAction(new Wait(1.0));
        LimelightProcessor processor = LimelightProcessor.getInstance();
        Drive drive = Drive.getInstance();
        double tx = processor.getTX();
        double yaw = processor.getYaw();
        double dist = 0.7* Math.hypot(processor.getX(), processor.getZ());
        double delta_x = 1.0 * dist / 12.0 * Math.cos(Math.toRadians(drive.getHeading() - tx));//-0.75 * processor.getZ()/ 12.0;//0.9 / 12.0 * processor.getDist() * Math.cos(Math.toRadians(drive.getHeading() - processor.getTX()));
        double delta_y = 1.0 * dist / 12.0 * Math.sin(Math.toRadians(drive.getHeading() - tx));
        List<Waypoint> path = Arrays.asList(new Waypoint(0.0, 0.0, Math.toRadians(drive.getHeading() - tx)), new Waypoint(delta_x,delta_y,Math.toRadians(drive.getHeading() - tx)));
        /*runAction(new SeriesAction(Arrays.asList(new OpenLoopDriveAction(-1.0, -1.0, 4.0), new OpenLoopDriveAction(1.0, 1.0, 4.0),
        new OpenLoopDriveAction(-1.0, -1.0, 4.0), new OpenLoopDriveAction(1.0, 1.0, 4.0), new OpenLoopDriveAction(-1.0, -1.0, 4.0),
        new OpenLoopDriveAction(1.0, 1.0, 4.0))));*/
        //runAction(new FollowTrajectoryAction(Arrays.asList(new Waypoint(0.0, 0.0, 0.0), new Waypoint(12.0, 6.25, Math.PI / 6.0)), false));
        //runAction(new FollowTrajectoryAction(FieldMeasurements.pathToCargoLeft, false));//Arrays.asList(new Waypoint(0.0, 0.0, 0.0), new Waypoint(12.0,-6.25,0.0)), false));
        //runAction(new FollowTrajectoryAction(Arrays.asList(new Waypoint(12.0, -6.25, Math.PI), new Waypoint(0.0,0.0, Math.PI)), true));
        //runAction(new TurnInPlaceAction(90.0, true, 1., 500.0));
        double yawAbs = Math.abs(yaw);
        double txAbs = Math.abs(tx);
        double sign = -Math.signum(yaw);
        double x = 1.0*Math.abs(processor.getX()) / 12.0;
        double y =  0.6* Math.abs(processor.getZ()) / 12.0;
        double a1 = -sign * (90 - txAbs - yawAbs);
        double a2 = sign * 90.0;
        double angle = sign * ( txAbs + yawAbs);

        List<Waypoint> path1 = Arrays.asList(new Waypoint(0.0, 0.0, Math.toRadians(a1)), new Waypoint(x * Math.cos(Math.toRadians(a1)),x * Math.sin(Math.toRadians(a1)),Math.toRadians(a1)));
        List<Waypoint> path2 = Arrays.asList(new Waypoint(0.0, 0.0, Math.toRadians(angle)), new Waypoint(y * Math.cos(Math.toRadians(angle)),y * Math.sin(Math.toRadians(angle)),Math.toRadians(angle)));
        
        if(Math.abs(x) >= 15.0) {
            runAction(new TurnInPlaceAction(a1, true, 1., 3.0)); 
            runAction(new FollowTrajectoryAction(path1, false));
            runAction(new TurnInPlaceAction(a2, true, 1., 3.0));
        } else {
            runAction(new TurnInPlaceAction(sign * txAbs, true, 1., 3.0)); 
        }
        runAction(new FollowTrajectoryAction(path2, false));
        /*runAction(new Wait(1.0));
        runAction(new IntakeAction(true, false));
        runAction(new FollowVisionAction());
        runAction(new Wait(0.5));
        runAction(new EjectPanelAction(true));
        runAction(new Wait(0.75));
        runAction(new Wait(0.5));
        runAction(new OpenLoopDriveAction(0.3, 0.3, 0.5));
        runAction(new EjectPanelAction(false));*/
        //runAction(new FollowTrajectoryAction(Arrays.asList(new Waypoint(0.0, 0.0, 0.0), new Waypoint(12.0, 0.0, 0.0)), false));
    }
}