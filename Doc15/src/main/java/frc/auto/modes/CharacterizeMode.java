package frc.auto.modes;

import java.util.ArrayList;
import java.util.List;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.CollectAccelerationData;
import frc.auto.actions.CollectVelocityData;
import physics.DriveCharacterization;

public class CharacterizeMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        //List<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList();
        List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();

        

        //runAction(new CollectVelocityData(velocityData, true, false, true));
        runAction(new CollectAccelerationData(accelerationData, true, false, true));

        
    }
}