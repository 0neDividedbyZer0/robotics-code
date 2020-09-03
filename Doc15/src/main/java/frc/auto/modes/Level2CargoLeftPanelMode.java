package frc.auto.modes;

import java.util.Arrays;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.FollowTrajectoryAction;
import frc.auto.actions.OpenLoopDriveAction;
import frc.auto.actions.SeriesAction;
import frc.auto.actions.TurnInPlaceAction;
import frc.util.FieldMeasurements;
import trajectory_lib.Waypoint;

public class Level2CargoLeftPanelMode extends AutoModeBase {

    @Override
    public void routine() throws AutoModeEndedException {
        
       runAction(new SeriesAction(Arrays.asList(new FollowTrajectoryAction(FieldMeasurements.pathToCargoLeft, false)))); 
    }
}