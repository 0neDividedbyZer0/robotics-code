package frc.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import drivers.VictorSPXFactory;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.loops.Loop;
import frc.loops.Looper;
import frc.robot.Constants;

public class Intake extends Subsystem {

    private static Intake instance;

    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    //Helper class for Intake
    public class IntakeState {
        public double velocity = 0.0;
        public boolean extend = false, eject = false, barOut = false;
        public boolean auto = false;
    }

    private VictorSPX intakeMotor;
    private Solenoid panelEjector, intakeBarExtender;
    private DoubleSolenoid intakeRetractors;
    private AnalogInput intakeBanner, ultraSonicSensor;
    private DigitalInput panelLimitSwitch;
    
    //State
    IntakeState wantedState = new IntakeState();
    IntakeState state = new IntakeState();
    private double panelEjectorTimestamp = 0.0;
    private boolean prevPanelEjector = false;
    private boolean keepEjected = false;
    private double distPerVolt = (14.0 - 11.0)/ (2.13 - 1.56);//(12.0 - 2.0)/ (2.5 - 0.015); // 11 in 1.56 V 2.13 V at 14 in
    
    

    private Intake() {
        intakeMotor = VictorSPXFactory.createDefaultVictor(Constants.INTAKE_VICTOR);
        intakeMotor.enableVoltageCompensation(true);
        intakeMotor.configVoltageCompSaturation(12.0, Constants.LONG_CAN_TIMEOUT_MS);
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        intakeRetractors = new DoubleSolenoid(Constants.RETRACTOR_DOUBLE_SOLENOID_1, Constants.RETRACTOR_DOUBLE_SOLENOID_2);
        panelEjector = new Solenoid(Constants.PANEL_SOLENOID);

        intakeBanner = new AnalogInput(Constants.INTAKE_BANNER);

        intakeBarExtender = new Solenoid(Constants.INTAKE_BAR_EXTENDER_SOL);

        panelLimitSwitch = new DigitalInput(Constants.INTAKE_LIMIT_SWITCH);

        ultraSonicSensor = new AnalogInput(Constants.ULTRA_SONIC_PORT);

    }

    Loop loop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            //intakeMotor.setNeutralMode(NeutralMode.Brake);
            
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(Intake.this) {
                handleIntake(wantedState, state, timestamp);
                
            } 
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    //IntakeBanner voltage needs tuning
    public boolean getBanner() {
        return intakeBanner.getVoltage() > 3.0;
    }

    private void handleIntake(IntakeState wanted, IntakeState current, double timestamp) {
        current.auto = wanted.auto;

        if(wanted.velocity == 0.0 && getBanner()) {
            setMotor(-0.33);
        } else {
            setMotor(wanted.velocity);
        }
        //setMotor(wanted.velocity);
        extendBar(wanted.barOut);
        extendIntake(wanted.extend);
        ejectPanel(wanted.eject, wanted.auto, timestamp);
    }

    public void setMotor(double vel) {
        intakeMotor.set(ControlMode.PercentOutput, vel);
    }

    public void extendBar(boolean out) {
        if(out != state.barOut) {
            state.barOut = out;
            if(out) {
                intakeBarExtender.set(true); //Check this 
            } else {
                intakeBarExtender.set(false);
            }
            
        }   
    }

    //extend should be down
    public void extendIntake(boolean extend) {
        if(extend != state.extend) {
            state.extend = extend;
            if(extend) {
                intakeRetractors.set(Value.kReverse); //Check this 
            } else {
                intakeRetractors.set(Value.kForward);
            }
            //System.out.println(intakeRetractors.get());
        }   
        
    }

    public Value intakeValue() {
        return intakeRetractors.get();
    }

    public void ejectPanel(boolean eject, boolean auto, double timestamp) {
        if(!auto) {
            if(eject != state.eject) {
                panelEjectorTimestamp = timestamp;
                state.eject = eject;
                
            }
    
            if(eject && timestamp - panelEjectorTimestamp > 0.1) {
                if(prevPanelEjector != true) {
                    prevPanelEjector = true;
                    panelEjector.set(true);
                }
            } else {
                if(prevPanelEjector != false && !keepEjected) {
                    prevPanelEjector = false;
                    panelEjector.set(false);
                }
            }
        } else {
            if(eject != state.eject) {
                state.eject = eject;
                if(eject) {
                    panelEjector.set(true);
                } else {
                    panelEjector.set(false);
                }
                
            }   
        }
        
        //panelEjector.set(eject);


        
    }

    public boolean getEjectorBoolean() {
        return panelEjector.get();
    }

    public boolean getLimitSwitch() {
        return panelLimitSwitch.get();
    }

    public IntakeState getState() {
        return state;
    }

    public synchronized void setWantedState(IntakeState wanted) {
        wantedState = wanted;
    }

    public void setKeepEjected(boolean nabors) {
        keepEjected = nabors;
    }

    public boolean getKeepEjected() {
        return keepEjected;
    }

    public IntakeState getWantedState() {
        return wantedState;
    }

    //Maybe need code for the ball
    public void outputToSmartDashboard() {
        SmartDashboard.putBoolean("Ball Detected", getBanner());

        SmartDashboard.putBoolean("Ejecting", getLimitSwitch());

        SmartDashboard.putNumber("Ultra Sonic sensor (inches)", getUltraSonicSensor());
    }

    public double getUltraSonicSensor() {
        return ultraSonicSensor.getVoltage() * distPerVolt + 2.5; 
    }

    public void stop() {
        setMotor(0.0);
    }

    public void zeroSensors() {

    }

    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(loop);
    }
}