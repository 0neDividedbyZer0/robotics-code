package frc.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.subsystems.Intake;
import frc.subsystems.Superstructure;

public class IntakeBallAction implements Action {
    private Intake intake = Intake.getInstance();
    private double power;
    private double timeout = 1.0;
    private double startTime;
    private boolean useBanner;

    public IntakeBallAction(double power, double timeout, boolean useBanner) {
        this.power = power;
        this.timeout = timeout;
        this.useBanner = useBanner;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();

        Intake.IntakeState wantedState = intake.new IntakeState();
        wantedState.velocity = power;

        wantedState.auto = true;
        wantedState.barOut = intake.getState().barOut;
        wantedState.extend = intake.getState().extend;
        wantedState.eject = intake.getState().eject;
       

        intake.setWantedState(wantedState);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        if(useBanner) {
            return intake.getBanner() || Timer.getFPGATimestamp() - startTime >= timeout;
        } else {
            return Timer.getFPGATimestamp() - startTime >= timeout;
        }
        
    }

    @Override
    public void done() {
        Intake.IntakeState wantedState = intake.new IntakeState();
        wantedState.velocity = 0.0;

        wantedState.auto = true;
        wantedState.barOut = intake.getState().barOut;
        wantedState.extend = intake.getState().extend;
        wantedState.eject = intake.getState().eject;

        intake.setWantedState(wantedState);
    }
    
}