package frc.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import control.ProfileConstraints;
import control.ProfileController;
import control.ProfilePoint;
import control.TrapezoidalProfile;
import drivers.VictorSPXFactory;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.loops.Loop;
import frc.loops.Looper;
import frc.robot.Constants;
import frc.statemachines.GadgetCommand;
import frc.statemachines.GadgetState;
import frc.statemachines.GadgetStateMachine;
import frc.statemachines.GadgetStateMachine.GadgetWantedState;
import frc.util.Util;

//Needs interruptibles
public class GoGoGadget extends Subsystem {

    private static GoGoGadget instance;

    public static GoGoGadget getInstance() {
        if(instance == null) {
            instance = new GoGoGadget();
        }
        return instance;
    }

    

    
    //Hardware
    private CANSparkMax frontActuator, rearActuator;

    private VictorSPX leftGadgetDrive, rightGadgetDrive;
    private AnalogInput proximityBanner;

    //Sensor Variables
    private double distPerTick;
    private double distPerVolt;
    private double frontOffset = 0.0, rearOffset = 0.0;

    //State
    private GadgetState observedState = new GadgetState();
    private GadgetStateMachine stateMachine = new GadgetStateMachine();
    private GadgetWantedState wantedState = GadgetWantedState.SYNCED;
    private boolean toleranceFlag;

    //Motion Profiles
    private ProfileController rearController; //Just a PID Controller now.


    private GoGoGadget() {
        frontActuator = new CANSparkMax(Constants.FRONT_NEO, MotorType.kBrushless);
        rearActuator = new CANSparkMax(Constants.REAR_NEO, MotorType.kBrushless);

        rearActuator.restoreFactoryDefaults();
        frontActuator.restoreFactoryDefaults();

        frontActuator.setIdleMode(IdleMode.kBrake);
        rearActuator.setIdleMode(IdleMode.kBrake);

        distPerTick = (3.8125 / 61.3614) * 22.0 /18.0; //PITCH / 42.0 It should be >:( but it isn't 3.8125 / 61.3614
        

        frontActuator.getEncoder().setPositionConversionFactor(distPerTick);
        rearActuator.getEncoder().setPositionConversionFactor(distPerTick * 1.013);

        frontActuator.getEncoder().setVelocityConversionFactor(distPerTick);
        rearActuator.getEncoder().setVelocityConversionFactor(distPerTick * 1.013);


        rightGadgetDrive = VictorSPXFactory.createDefaultVictor(Constants.RIGHT_GADGET_DRIVE_VICTOR);
        leftGadgetDrive = VictorSPXFactory.createDefaultVictor(Constants.LEFT_GADGET_DRIVE_VICTOR);
        
        rightGadgetDrive.enableVoltageCompensation(true);
        leftGadgetDrive.enableVoltageCompensation(true);

        rightGadgetDrive.configVoltageCompSaturation(12.0, Constants.LONG_CAN_TIMEOUT_MS);
        leftGadgetDrive.configVoltageCompSaturation(12.0, Constants.LONG_CAN_TIMEOUT_MS);

        leftGadgetDrive.setNeutralMode(NeutralMode.Brake);
        rightGadgetDrive.setNeutralMode(NeutralMode.Brake);

        proximityBanner = new AnalogInput(Constants.PROXIMITY_BANNER);

        

        distPerVolt = 1.0; // Needs to be set

        

        rearController = new ProfileController(Constants.REAR_GADGET_KP, Constants.REAR_GADGET_KI, Constants.REAR_GADGET_KD, Constants.REAR_GADGET_KV);
        rearController.setOutputRange(-.4, 0.4);
        toleranceFlag = false;
    }


    private Loop loop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stateMachine.resetDrive();
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(GoGoGadget.this) {
               updateObservedState();

               GadgetCommand command = stateMachine.update(timestamp, wantedState, observedState);

               setFromCommand(timestamp, command);
                
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            
        }
    };

    private void setFromCommand(double timestamp, GadgetCommand command) {
        
        if(command.synced) {
            double power = handleSynchronized(command);
            //A little hacky :P
            ProfilePoint point = new ProfilePoint(observedState.frontActuatorLength , 0.0);
            double compensation = rearController.calculate(point, observedState.rearActuatorLength, Constants.LOOPER_DT);
            if(toleranceFlag && Util.epsilonEquals(observedState.frontActuatorLength, observedState.rearActuatorLength, 0.1)) {
                compensation = 0.0;
            }
            setFrontMotor(2.0 * power);
            setRearMotor((1.83 * power + compensation));

            setGadgetDrive(command.velocity);
        } else {
            if(command.openLoop) {
                
            } else {
                double frontPower = handleUnsynchronizedFront(command);
                double rearPower = handleUnsynchronizedRear(command);

                

                setFrontMotor(frontPower);
                setRearMotor(rearPower);

                setGadgetDrive(0.0);
            }
            
        }
        
    }

    private void updateObservedState() {
        observedState.frontActuatorLength = getFrontEncoder();
        observedState.rearActuatorLength = getRearEncoder();
        observedState.frontActuatorSpeed = getFrontEncoderRate();
        observedState.rearActuatorSpeed = getRearEncoderRate();
    }

    private double handleSynchronized(GadgetCommand command) {
        //Tolerance needs some tuning
        double currentHeight = observedState.frontActuatorLength;
        double wantedHeight = command.frontActuatorHeight;
        //double currentRearHeight = observedState.rearActuatorLength;
        double wantedPower = 0.0;
        if(checkTolerance(currentHeight, wantedHeight, .5)) {
            wantedPower = 0.0;
            toleranceFlag = true;
        } else {
            toleranceFlag = false;
            if(currentHeight < wantedHeight) {
                wantedPower = 0.5;
            } else if(currentHeight > wantedHeight) {
                wantedPower = -0.5;
            }
        }
        return wantedPower;

    }

    private double handleUnsynchronizedFront(GadgetCommand command) {
        //Tolerance needs some tuning
        double currentHeight = observedState.frontActuatorLength;
        double wantedHeight = command.frontActuatorHeight;
        double wantedPower = 0.0;
        if(checkTolerance(currentHeight, wantedHeight, .5)) {
            wantedPower = 0.0;
        } else {
            if(currentHeight < wantedHeight) {
                wantedPower = 1.0;
            } else if(currentHeight > wantedHeight) {
                wantedPower = -1.0;
            }
        }
        return wantedPower;

    }

    private double handleUnsynchronizedRear(GadgetCommand command) {
        //Tolerance needs some tuning
        double currentHeight = observedState.rearActuatorLength;
        double wantedHeight = command.rearActuatorHeight;
        double wantedPower = 0.0;
        if(checkTolerance(currentHeight, wantedHeight, 0.5)) {
            wantedPower = 0.0;
            
        } else {
            if(currentHeight < wantedHeight) {
                wantedPower = 1.0;
            } else if(currentHeight > wantedHeight) {
                wantedPower = -1.0;
            }
            
        }
        return wantedPower;

    }

    //Please make tolerance positive
    private boolean checkTolerance(double value1, double value2, double tolerance) {
        return Math.abs(value1 - value2) < tolerance;
    }
		
    
    public GadgetState getObservedState() {
        return observedState;
    }

    public synchronized void setWantedState(GadgetWantedState wanted) {
        wantedState = wanted;
    }

    public synchronized GadgetWantedState getWantedState() {
        return wantedState;
    }

    public synchronized void setGadgetDrive(double speed) {
        rightGadgetDrive.set(ControlMode.PercentOutput, speed);
        leftGadgetDrive.set(ControlMode.PercentOutput, speed);
    }

    public synchronized void setFrontMotor(double speed) {
        frontActuator.set(speed);
    }

    public synchronized void setRearMotor(double speed) {
        rearActuator.set(speed);
    }

    public synchronized void setFrontGoal(double frontHeight) {
        stateMachine.setFrontActuatorHeight(frontHeight);
    }

    public synchronized void setRearGoal(double rearHeight) {
        stateMachine.setRearActuatorHeight(rearHeight);
    }

    public double getFrontEncoder() {
        return frontActuator.getEncoder().getPosition() - frontOffset;
    }

    public double getRearEncoder() {
        return rearActuator.getEncoder().getPosition() - rearOffset;
    }

    public double getFrontEncoderRate() {
        return frontActuator.getEncoder().getVelocity();
    }

    public double getRearEncoderRate() {
        return rearActuator.getEncoder().getVelocity();
    }

    public double ticksToDist(double ticks) {
        return ticks * distPerTick;
    }

    

    public void resetEncoders() {
        frontOffset = frontActuator.getEncoder().getPosition();
        rearOffset = rearActuator.getEncoder().getPosition();
    }

    public boolean getProximity() {
        return proximityBanner.getVoltage() > 3.0; //When activated its about 5.0
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Front Gadget Encoder (inches)", getFrontEncoder());
        SmartDashboard.putNumber("Rear Gadget Encoder (inches)", getRearEncoder());

        SmartDashboard.putNumber("Front Gadget Encoder Rate (ips)", getFrontEncoderRate());
        SmartDashboard.putNumber("Rear Gadget Encoder Rate (ips)", getRearEncoderRate());

        
        SmartDashboard.putBoolean("Proximity Banner", getProximity());

        //SmartDashboard.putNumber("Current Rear", rearActuator.getOutputCurrent());

        //SmartDashboard.putNumber("Current Front", frontActuator.getOutputCurrent());
        
    }

    public void stop() {
        setGadgetDrive(0.0);
        frontActuator.stopMotor();
        rearActuator.stopMotor();
    }

    public void zeroSensors() {
        resetEncoders();
    }

    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(loop);
    }
}