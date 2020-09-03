package frc.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.subsystems.Drive;
import frc.util.DriveSignal;

public class OpenLoopDriveAction implements Action {
    private Drive drive = Drive.getInstance();
    private double leftPow, rightPow;
    private double timeout;
    private double startTime;

    public OpenLoopDriveAction(double leftPow, double rightPow, double timeout) {
        this.leftPow = leftPow;
        this.rightPow = rightPow;
        this.timeout = timeout;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
        drive.setOpenLoop(new DriveSignal(leftPow, rightPow));
    }

    @Override 
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= timeout;
    }

    @Override
    public void done() {
        drive.setOpenLoop(DriveSignal.NEUTRAL);
    }

}