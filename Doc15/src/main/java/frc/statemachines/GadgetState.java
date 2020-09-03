package frc.statemachines;

public class GadgetState {
    public double frontActuatorLength, rearActuatorLength;
    public double frontActuatorSpeed, rearActuatorSpeed;
    public double ultraSonic;
    public boolean proximity;


    public GadgetState() {
        frontActuatorLength = 0.0;
        rearActuatorLength = 0.0; 
        frontActuatorSpeed = 0.0;
        rearActuatorSpeed = 0.0;
        
        proximity = false;
    }


}