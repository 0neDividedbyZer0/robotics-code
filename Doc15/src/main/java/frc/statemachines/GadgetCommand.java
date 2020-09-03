package frc.statemachines;

import frc.robot.Constants;

//Helper class for GoGoGadget
public class GadgetCommand {
    
    public double frontActuatorHeight = Constants.MIN_LENGTH, rearActuatorHeight = Constants.MIN_LENGTH;
    public double velocity = 0.0;
    public boolean synced = false; 
    public boolean openLoop = false;



    public GadgetCommand(double frontActuatorHeight, double rearActuatorHeight, double velocity, boolean openLoop) {
        this.frontActuatorHeight = frontActuatorHeight;
        this.rearActuatorHeight = rearActuatorHeight;
        this.velocity = velocity;
        this.openLoop = openLoop;
    }

    public GadgetCommand(double frontHeight, double rearHeight) {
        this(frontHeight, rearHeight, 0.0, false);
    }

    public GadgetCommand() {

    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }


}