package frc.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.loops.Loop;
import frc.loops.Looper;
import frc.robot.Constants;

public class LimelightProcessor extends Subsystem {
    private static LimelightProcessor instance;

    public static LimelightProcessor getInstance() {
        if(instance == null) {
            instance = new LimelightProcessor();
        }
        return instance;
    }

    public enum DetectionMode {
        HATCH,
        CARGO
    }

    private NetworkTable table;
    private NetworkTableEntry ledMode, webCamMode, tx, ty, ts, tv, camtran;
    private double t_x, t_y, t_s, t_v, dist;
    private double x, y, z, yaw, pitch, roll;
    private int currentLedMode, currentWebCamMode;
    private DetectionMode detectionMode;

   
    

    private LimelightProcessor() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        ledMode = table.getEntry("ledMode");
        //webCamMode = table.getEntry("camMode");
        camtran = table.getEntry("camtran");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ts = table.getEntry("ts");
        tv = table.getEntry("tv");
        

        detectionMode = DetectionMode.HATCH;
    }

    Loop loop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            ledMode.setDouble(currentLedMode);
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(LimelightProcessor.this) {
                handleProcessing();
            } 
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    public synchronized void setLedMode(int wantedLedMode) {
        if(wantedLedMode != currentLedMode) {
            currentLedMode = wantedLedMode;
            ledMode.setDouble(currentLedMode);
        }
    }

    public synchronized void setWebCamMode(int wantedWebCamMode) {
        if(wantedWebCamMode != currentWebCamMode) {
            currentWebCamMode = wantedWebCamMode;
            //webCamMode.setDouble(currentWebCamMode);
        }
    }

    public synchronized void setDetectionMode(DetectionMode wantedDetectionMode) {
        if(detectionMode != wantedDetectionMode) {
            detectionMode = wantedDetectionMode;
        }
    }

    private void processDist() {
        
        
        if(detectionMode == DetectionMode.HATCH) {
            dist = (Constants.h3 - Constants.h1) / Math.tan(Math.toRadians(Constants.a1 + t_y));
        } else {
            dist = (Constants.h2 - Constants.h1) / Math.tan(Math.toRadians(Constants.a1 + t_y));
        }
    }

    public double getCalibrationAngle() {
        if(detectionMode == DetectionMode.HATCH) {
            //tan(a1+a2)=deltah/d
            return Math.toDegrees(Math.atan((Constants.h3-Constants.h1)/ 38.5)) - t_y;
        } else {
            return Math.toDegrees(Math.atan((Constants.h2-Constants.h1)/ 38.5)) - t_y;
        }
    }

    

    private void handleProcessing() {
        t_v = tv.getDouble(0.0);
        if(t_v == 1.0) {
            t_x = tx.getDouble(0.0);
            t_y = ty.getDouble(0.0);
            t_s = ts.getDouble(0.0);
            processDist();

            /*double[] orientation = camtran.getDoubleArray(new double[] {0.0 ,0.0 ,0.0 ,0.0, 0.0 ,0.0});
            x = orientation[0];
            y = orientation[1];
            z = orientation[2];
            pitch = orientation[3];
            yaw = orientation[4];
            roll = orientation[5];*/
        }
        
    }

    public synchronized double getX() {
        return x;
    }

    public synchronized double getY() {
        return y;
    }

    public synchronized double getZ() {
        return z;
    }

    public synchronized double getYaw() {
        return yaw;
    }

    public synchronized double getPitch() {
        return pitch;
    }

    public synchronized double getRoll() {
        return roll;
    }
    
    public synchronized double getTX() {
        return t_x;
    }

    public synchronized double getTY() {
        return t_y;
    }

    public synchronized double getTS() {
        return t_s;
    }

    public synchronized double getTV() {
        return t_v;
    }

    public synchronized double getDist() {
        return dist;
    }
    

    @Override
    public void outputToSmartDashboard() {
        /*SmartDashboard.putNumber("a1", getCalibrationAngle());
        SmartDashboard.putNumber("tx", t_x);
        SmartDashboard.putNumber("ty", t_y);
        SmartDashboard.putNumber("tv", t_v);
        SmartDashboard.putNumber("ts", t_s);
        SmartDashboard.putNumber("dist", dist);

        SmartDashboard.putNumber("x", x);
        SmartDashboard.putNumber("y", y);
        SmartDashboard.putNumber("z", z);
        SmartDashboard.putNumber("yaw", yaw);
        SmartDashboard.putNumber("pitch", pitch);
        SmartDashboard.putNumber("roll", roll);*/
        
    }

    @Override
    public void stop() {
        ledMode.setDouble(1);
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(loop);
    }
}