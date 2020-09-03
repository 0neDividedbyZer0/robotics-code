package frc.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.subsystems.Intake;
import frc.subsystems.Superstructure;

public class EjectPanelAction implements Action {
    private Intake intake = Intake.getInstance();
    private boolean eject;

    public EjectPanelAction(boolean eject) {
        this.eject = eject;
    }

    @Override
    public void start() {

        Intake.IntakeState wantedState = intake.getWantedState();

        wantedState.eject = eject;

        wantedState.velocity = intake.getWantedState().velocity;
        wantedState.auto = true;
        wantedState.barOut = intake.getWantedState().barOut;
        wantedState.extend = intake.getWantedState().extend;
       
       

        intake.setWantedState(wantedState);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return true;
        
    }

    @Override
    public void done() {
    
    }
    
}