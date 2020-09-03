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

public class CargoRightPanelLevel2Mode extends AutoModeBase {

    @Override
    public void routine() throws AutoModeEndedException {
        
        runAction(new SeriesAction(Arrays.asList(new FollowTrajectoryAction(FieldMeasurements.pathToCargoRight, false), new EjectPanelAction(true)))); 
        //SeriesAction retractPanel = new SeriesAction(Arrays.asList(new Wait(0.5), new EjectPanelAction(false)));
        //runAction(new ParallelAction(Arrays.asList(new FollowTrajectoryAction(Arrays.asList(new Waypoint(0.0, 0.0, Math.PI), new Waypoint(-3.0, 0.0, Math.PI)), true), retractPanel)));
    }
}