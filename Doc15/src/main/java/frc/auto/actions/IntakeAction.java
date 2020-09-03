package frc.auto.actions;

import frc.subsystems.Intake;
import frc.subsystems.Superstructure;

public class IntakeAction implements Action {
    private Intake intake = Intake.getInstance();
    private boolean barOut, extendIntake;

    public IntakeAction(boolean barOut, boolean extendIntake) {
        this.barOut = barOut;
        this.extendIntake = extendIntake;
    }

    @Override
    public void start() {
        Intake.IntakeState wantedState = intake.getWantedState();
        
        wantedState.barOut = barOut;
        wantedState.extend = extendIntake;
        wantedState.auto = true;

        wantedState.eject = intake.getWantedState().eject;
        wantedState.velocity = intake.getWantedState().velocity;

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