package frc.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import drivers.VictorSPXFactory;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.loops.Loop;
import frc.loops.Looper;
import frc.robot.Constants;
import frc.util.DriveSignal;
import frc.util.Util;
import trajectory_lib.AutoTrajectory;
import trajectory_lib.HeadingController;
import trajectory_lib.TrajectoryConfig;
import trajectory_lib.TrajectoryFollower;
import trajectory_lib.Utils;

public class Drive extends Subsystem {

    private static Drive instance;

    public static Drive getInstance() {
        if(instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    public enum DriveState {
        OPEN_LOOP,
        PATH_FOLLOWING,
        TURN_IN_PLACE,
        PURE_PURSUIT
    }

    //Hardware
    private VictorSPX leftFront, leftRear, rightFront, rightRear;
    private Encoder leftEncoder, rightEncoder;
    private Solenoid shifter;
    //private PigeonIMU imu;
    //private ADXRS450_Gyro spartanBoard;

    //State Variables
    private boolean isHighGear;
    private boolean isBrakeMode;
    private double angleOffset = 0.0;

    //Control: Path Following
    private TrajectoryFollower trajectoryFollower;
    private TrajectoryConfig config, testConfig;
    private boolean reversed = false;

    //Control: Turn in Place
    private double absAngleSetpoint;
    private HeadingController headingController;

    //State
    private DriveState driveState = DriveState.OPEN_LOOP;

    private Drive() {
        leftFront = VictorSPXFactory.createDefaultVictor(Constants.LEFT_FRONT_DRIVE_VICTOR);
        leftRear = VictorSPXFactory.createPermanentSlaveVictor(Constants.LEFT_REAR_DRIVE_VICTOR, Constants.LEFT_FRONT_DRIVE_VICTOR);

        rightFront = VictorSPXFactory.createDefaultVictor(Constants.RIGHT_FRONT_DRIVE_VICTOR);
        rightRear = VictorSPXFactory.createPermanentSlaveVictor(Constants.RIGHT_REAR_DRIVE_VICTOR, Constants.RIGHT_FRONT_DRIVE_VICTOR);
        
        leftFront.enableVoltageCompensation(true);
        rightFront.enableVoltageCompensation(true);

        leftFront.configVoltageCompSaturation(12.0, Constants.LONG_CAN_TIMEOUT_MS);
        rightFront.configVoltageCompSaturation(12.0, Constants.LONG_CAN_TIMEOUT_MS);

        //0.1
        leftFront.configOpenloopRamp(0., Constants.LONG_CAN_TIMEOUT_MS);
        rightFront.configOpenloopRamp(0., Constants.LONG_CAN_TIMEOUT_MS);

        leftEncoder = new Encoder(Constants.DRIVE_LEFT_ENCODER_2, Constants.DRIVE_LEFT_ENCODER_1);
        rightEncoder = new Encoder(Constants.DRIVE_RIGHT_ENCODER_1, Constants.DRIVE_RIGHT_ENCODER_2);

        leftEncoder.setDistancePerPulse(Constants.WHEEL_DIAMETER * Math.PI / 360.0);//0.972
        rightEncoder.setDistancePerPulse(Constants.WHEEL_DIAMETER * Math.PI / 360.0);

        /*imu = new PigeonIMU(Constants.DRIVE_IMU);

        spartanBoard = new ADXRS450_Gyro(Port.kOnboardCS0);
        spartanBoard.calibrate();*/

        shifter = new Solenoid(Constants.SHIFTER_SOLENOID);

        isHighGear = true;

        isBrakeMode = true;

        absAngleSetpoint = 0.0;

        config = new TrajectoryConfig(TrajectoryConfig.SAMPLES_FAST, 0.005, Constants.DRIVE_ALLOWED_SPEED, Constants.DRIVE_MAX_ACC, Constants.DRIVE_MAX_JERK);

        testConfig = new TrajectoryConfig(TrajectoryConfig.SAMPLES_FAST, 0.005, Constants.TEST_ALLOWED_SPEED, Constants.DRIVE_MAX_ACC, Constants.DRIVE_MAX_JERK);

        trajectoryFollower = new TrajectoryFollower(Constants.TRAJECTORY_FOLLOWER_KP, Constants.TRAJECTORY_FOLLOWER_KI, 
            Constants.TRAJECTORY_FOLLOWER_KD, Constants.TRAJECTORY_FOLLOWER_KV, Constants.TRAJECTORY_FOLLOWER_KA, 
            Constants.TRAJECTORY_FOLLOWER_KW);

        trajectoryFollower.getHeadingController().setInputRange(-Math.PI, Math.PI);
        trajectoryFollower.getHeadingController().setPID(Constants.HEADING_CONTROLLER_KP, Constants.HEADING_CONTROLLER_KI, Constants.HEADING_CONTROLLER_KD);

        headingController = new HeadingController(Constants.HEADING_CONTROLLER_KP, Constants.HEADING_CONTROLLER_KI,
            Constants.HEADING_CONTROLLER_KD);

        //headingController.setOutputRange(-0.7, 0.7);

        
    }

    Loop loop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(Drive.this)  {
                switch(driveState) {
                    case OPEN_LOOP:
                        return;
                    case PATH_FOLLOWING:
                        handlePathFollowing();
                        return;
                    case TURN_IN_PLACE:
                        handleTurnInPlace();
                        return;
                    default:
                        System.out.println("Unexpected Drive Control State: " + driveState);
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    public synchronized TrajectoryConfig getTestConfig() {
        return testConfig;
    }

    public synchronized void setRampRate(double rampRate) {
        leftFront.configOpenloopRamp(rampRate, Constants.LONG_CAN_TIMEOUT_MS);
        rightFront.configOpenloopRamp(rampRate, Constants.LONG_CAN_TIMEOUT_MS);
    }

    //Degrees please!!!
    public synchronized void setAbsoluteAngle(double angleToSet) {
        absAngleSetpoint = angleToSet;
        headingController.reset();
    }

    public synchronized void setRelativeAngle(double relativeAngle) {
        absAngleSetpoint = getHeading() + relativeAngle;
        headingController.reset();
    }

    public synchronized double getAbsAngleSetpoint() {
        return absAngleSetpoint;
    }

    private void handleTurnInPlace() {
        double powerToTurn = headingController.calculate(Utils.boundHalfDegrees(absAngleSetpoint), Utils.boundHalfDegrees(getHeading()), Constants.LOOPER_DT);
        DriveSignal signal = new DriveSignal(powerToTurn, -powerToTurn); 
        if(Util.epsilonEquals(getHeading(), absAngleSetpoint, 0.5)) {
            signal = DriveSignal.NEUTRAL;
            //SmartDashboard.putBoolean("Aim Finished", true);
        } else {
            //SmartDashboard.putBoolean("Aim Finished", false);
        }
        
        setMotors(signal);
    }

    public void setTrajectory(AutoTrajectory traj, boolean reversed) {
        trajectoryFollower.setTrajectory(traj);
        this.reversed = reversed;
    }

    public synchronized void startPathFollowing() {
        if(driveState != DriveState.PATH_FOLLOWING) {
            driveState = DriveState.PATH_FOLLOWING;
            trajectoryFollower.start(inToFt(getLeftEncoder()), inToFt(getRightEncoder()), reversed);
            
        }
    }

    private void handlePathFollowing() {
        DriveSignal signal = trajectoryFollower.calculate(inToFt(getLeftEncoder()), inToFt(getLeftEncoderRate()), inToFt(getRightEncoder()), 
            inToFt(getRightEncoderRate()), getHeading(), Constants.LOOPER_DT);
        DriveSignal bob = new DriveSignal(-signal.getLeft(), -signal.getRight());
        
        setMotors(bob);
        //System.out.println(bob.getLeft());
    }

    

    public synchronized void setDriveState(DriveState wantedState) {
        if(driveState != wantedState) {
            driveState = wantedState;
        }
    }

    public DriveState getDriveState() {
        return driveState;
    }

    public void setMotors(DriveSignal signal) {
        leftFront.set(ControlMode.PercentOutput, signal.getLeft());
        rightFront.set(ControlMode.PercentOutput, signal.getRight());
        SmartDashboard.putNumber("signal", signal.getLeft());
        
    }

    public synchronized void setHighGear(boolean on) {
        if(isHighGear != on) {
            isHighGear = on;
            if(on) {
                shifter.set(false);
            } else {
                shifter.set(true);
            }
        }
    }

    public synchronized void setBrakeMode(boolean on) {
        if(isBrakeMode != on) {
            isBrakeMode = on;
            if(on) {
                leftFront.setNeutralMode(NeutralMode.Brake);
                leftRear.setNeutralMode(NeutralMode.Brake);

                rightFront.setNeutralMode(NeutralMode.Brake);
                rightRear.setNeutralMode(NeutralMode.Brake);
            } else {
                leftFront.setNeutralMode(NeutralMode.Coast);
                leftRear.setNeutralMode(NeutralMode.Coast);

                rightFront.setNeutralMode(NeutralMode.Coast);
                rightRear.setNeutralMode(NeutralMode.Coast);
            }
        }
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if(driveState != DriveState.OPEN_LOOP) {
            driveState = DriveState.OPEN_LOOP;
            setBrakeMode(false);
            
        }
        //signal = new DriveSignal(1.0, 1.0);
        setMotors(signal);
    }

    public void zeroEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void zeroGyro() {
        /*PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
        angleOffset = imu.getFusedHeading(fusionStatus);
        spartanBoard.reset();*/
    }

    private double inToFt(double inches) {
        return inches / 12.0;
    }

    public double getLeftEncoder() {
        return leftEncoder.getDistance();
    }

    public double getRightEncoder() {
        return rightEncoder.getDistance();
    }

    public double getLeftEncoderRate() {
        return leftEncoder.getRate();
    }

    public double getRightEncoderRate() {
        return rightEncoder.getRate();
    }

    public double getHeading() {
        //PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
        return 0.0;//imu.getFusedHeading(fusionStatus) - angleOffset;
        //return spartanBoard.getAngle();
    }

    public double getHeadingRate() {
        double[] rpy = new double[3];
		//imu.getRawGyro(rpy);
        return 0.0;//rpy[2];
        //return spartanBoard.getRate();
    }

    public double getPitch() {
        double[] rpy = new double[3];
        //imu.getRawGyro(rpy);
        return 0.0;//rpy[1];
    }

    public TrajectoryConfig getConfig() {
        return config;
    }

    public boolean pathIsFinished() {
        return trajectoryFollower.isFinished();
    }

    @Override
    public void outputToSmartDashboard() {
        /*SmartDashboard.putNumber("Left Drive (in)", getLeftEncoder());
        SmartDashboard.putNumber("Right Drive (in)", getRightEncoder());

        SmartDashboard.putNumber("Left Drive Rate (ips)", getLeftEncoderRate());
        SmartDashboard.putNumber("Right Drive Rate (ips)", getRightEncoderRate());

        SmartDashboard.putNumber("Heading (deg)", getHeading());
        SmartDashboard.putNumber("Heading Rate (deg per sec)", getHeadingRate());
        SmartDashboard.putNumber("Pitch (deg)", getPitch());*/

        
    }


    @Override
    public void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void zeroSensors() {
        zeroEncoders();
        zeroGyro();
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(loop);
    }
}