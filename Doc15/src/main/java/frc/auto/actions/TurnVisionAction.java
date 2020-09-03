package frc.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.subsystems.Drive;
import frc.subsystems.LimelightProcessor;
import frc.subsystems.Drive.DriveState;
import frc.util.DriveSignal;
import trajectory_lib.Utils;

public class TurnVisionAction implements Action {
    private LimelightProcessor processor = LimelightProcessor.getInstance();
    private Drive drive = Drive.getInstance();
    private double tolerance = 1.0;
    private double timeout = 2.0;
    private double startTime;

    //Degrees
    public TurnVisionAction(boolean relative, double tolerance, double timeout) {
        if(relative) {
            drive.setRelativeAngle(-processor.getTX());
        } else {
            drive.setAbsoluteAngle(-processor.getTX());
        }
        this.tolerance = tolerance;
        this.timeout = timeout;
    }

    
    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
        drive.setDriveState(DriveState.TURN_IN_PLACE);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getHeading() - drive.getAbsAngleSetpoint()) < tolerance
            || Timer.getFPGATimestamp() - startTime >= timeout;
    }

    @Override
    public void done() {
        drive.setOpenLoop(DriveSignal.NEUTRAL);
    }
}