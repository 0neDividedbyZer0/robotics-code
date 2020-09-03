package frc.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import control.ProfileConstraints;
import control.ProfileController;
import control.ProfilePoint;
import control.TrapezoidalProfile;
import drivers.VictorSPXFactory;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.loops.Loop;
import frc.loops.Looper;
import frc.robot.Constants;

public class Elevator extends Subsystem {

    private static Elevator instance;

    public static Elevator getInstance() {
        if(instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    public enum ElevatorState {
        OPEN_LOOP,
        AUTO
    }

    //Hardware
    private VictorSPX leftElevator, rightElevator;
    private Encoder leftEncoder; //, rightEncoder;

    //Control
    private ProfilePoint updatingGoal, finalGoal;
    private ProfileConstraints constraints;
    private ProfileController controller;

    //System
    private ElevatorState state = ElevatorState.OPEN_LOOP;
    private boolean isBrakeMode;

    private Elevator() {
        leftElevator = VictorSPXFactory.createDefaultVictor(Constants.LEFT_ELEVATOR_VICTOR);
        rightElevator = VictorSPXFactory.createPermanentSlaveVictor(Constants.RIGHT_ELEVATOR_VICTOR, Constants.LEFT_ELEVATOR_VICTOR);

        leftElevator.enableVoltageCompensation(true);
        leftElevator.configVoltageCompSaturation(12.0, Constants.LONG_CAN_TIMEOUT_MS);

        leftEncoder = new Encoder(Constants.LEFT_ELEV_ENC_1, Constants.LEFT_ELEV_ENC_2);
        //rightEncoder = new Encoder(Constants.RIGHT_ELEV_ENC_1, Constants.RIGHT_ELEV_ENC_2);

        leftEncoder.setDistancePerPulse(35.0 / 1170.5);//Constants.PULLEY_DIAMETER * Math.PI / 360.0); // 35 in / 1170.5 pulse
        //rightEncoder.setDistancePerPulse(Constants.PULLEY_DIAMETER * Math.PI / 360.0);

        leftEncoder.setReverseDirection(true);

        constraints = new ProfileConstraints(Constants.ELEV_MAX_ALLOWED_LIN_VEL, Constants.ELEV_MAX_LIN_ACC);
        updatingGoal = new ProfilePoint(0.0, 0.0);
        finalGoal = new ProfilePoint(0.0, 0.0);
        controller = new ProfileController(Constants.ELEV_KP, Constants.ELEV_KI, Constants.ELEV_KD, Constants.ELEV_KV);

        isBrakeMode = false;

    }

    private Loop loop = new Loop() {
        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(Elevator.this) {
                switch(state) {
                    case OPEN_LOOP:
                        return;
                    case AUTO:
                        handleAuto();
                        return;
                    default:
                        System.out.println("Unexpected Elevator Control State: " + state);
                }
            }
        }
    
        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    public synchronized void setOpenLoop(double signal) {
        if(state != ElevatorState.OPEN_LOOP) {
            state = ElevatorState.OPEN_LOOP;
            setBrakeMode(false);
        }
        setMotors(signal);
        
    }

    private void setBrakeMode(boolean brake) {
        if(brake != isBrakeMode) {
            isBrakeMode = brake;
            if(brake) {
                leftElevator.setNeutralMode(NeutralMode.Brake);
                rightElevator.setNeutralMode(NeutralMode.Brake);
            } else {
                leftElevator.setNeutralMode(NeutralMode.Coast);
                rightElevator.setNeutralMode(NeutralMode.Coast);
            }
        }

    }

    //Please make max > min dummy
    public synchronized void setGoal(double wantedHeight) {
        wantedHeight = Math.max(Math.min(wantedHeight, Constants.ELEVATOR_MAX), Constants.ELEVATOR_MIN);
        finalGoal = new ProfilePoint(wantedHeight, 0.0);
        startAuto();
    }

    private synchronized void startAuto() {
        if(state != ElevatorState.AUTO) {
            state = ElevatorState.AUTO;
            //Careful, if left encoder fails we are f***ed
            updatingGoal = new ProfilePoint(getLeftEncoder(), getLeftEncoderRate());
            //setBrakeMode(true);
        }
        
    }

    private synchronized void setPID(double kp, double ki, double kd, double kv) {
        controller.setPID(kp, ki, kd, kv);
    }

    private void handleAuto() {
        ProfilePoint goalToSet = updateProfile();
        double currentPos = getLeftEncoder();
        
        double signal = controller.calculate(goalToSet, currentPos, Constants.LOOPER_DT);
        
        
        setMotors(signal);
    }

    private ProfilePoint updateProfile() {
        TrapezoidalProfile profile = new TrapezoidalProfile(constraints, finalGoal, updatingGoal);
        updatingGoal = profile.calculate(Constants.LOOPER_DT);
        return updatingGoal;
    }

    public double getGoal() {
        return finalGoal.getPos();
    }

    public void setMotors(double signal) {
        leftElevator.set(ControlMode.PercentOutput, signal);
    }

    public double getLeftEncoder() {
        return leftEncoder.getDistance();
    }

    /*public double getRightEncoder() {
        return rightEncoder.getDistance();
    }*/

    public double getLeftEncoderRate() {
        return leftEncoder.getRate();
    }

    /*public double getRightEncoderRate() {
        return rightEncoder.getRate();
    }*/

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Left Enc (in)", getLeftEncoder());
        //SmartDashboard.putNumber("Right Enc (in)", getRightEncoder());

        SmartDashboard.putNumber("Left Enc Rate(ips)", getLeftEncoderRate());
        //SmartDashboard.putNumber("Right Enc Rate(ips)", getRightEncoderRate());

        /*SmartDashboard.putNumber("Elev kp", controller.getP());
        SmartDashboard.putNumber("Elev ki", controller.getI());
        SmartDashboard.putNumber("Elev kd", controller.getD());
        SmartDashboard.putNumber("Elev kv", controller.getV());*/

        
    }

    public void stop() {
        setOpenLoop(0.0);
    }

    public void zeroSensors() {
        leftEncoder.reset();
        //rightEncoder.reset();

    }

    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(loop);
    }
}