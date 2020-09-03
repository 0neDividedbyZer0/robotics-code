package frc.auto.modes;

import java.util.Arrays;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.EjectPanelAction;
import frc.auto.actions.FollowTrajectoryAction;
import frc.auto.actions.OpenLoopDriveAction;
import frc.auto.actions.ParallelAction;
import frc.auto.actions.SeriesAction;
import frc.auto.actions.TurnInPlaceAction;
import frc.auto.actions.Wait;
import frc.util.FieldMeasurements;
import trajectory_lib.Waypoint;

public class CargoLeftPanelLevel2Mode extends AutoModeBase {

    @Override
    public void routine() throws AutoModeEndedException {
        //runAction(new OpenLoopDriveAction(-0.7, -0.7, 3.0));
        runAction(new FollowTrajectoryAction(Arrays.asList(new Waypoint(0.0, 0.0, 0.0),new Waypoint(6.0, 0.0, 0.0), new Waypoint(12.5, -2.75, 0.0)), false));
        //runAction(new TurnInPlaceAction(60.0, true, 2.0));
        //runAction(new TurnInPlaceAction(60.0, true, 2.0));
        //runAction(new SeriesAction(Arrays.asList(new FollowTrajectoryAction(FieldMeasurements.pathToCargoLeft, false), new EjectPanelAction(true)))); 
        //SeriesAction retractPanel = new SeriesAction(Arrays.asList(new Wait(0.9), new EjectPanelAction(false)));
        //runAction(new ParallelAction(Arrays.asList(new FollowTrajectoryAction(Arrays.asList(new Waypoint(0.0, 0.0, Math.PI), new Waypoint(-3.0, 0.0, Math.PI)), true), retractPanel)));
    }
}