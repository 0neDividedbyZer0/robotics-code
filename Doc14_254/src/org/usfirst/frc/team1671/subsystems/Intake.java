package org.usfirst.frc.team1671.subsystems;

import org.usfirst.frc.team1671.loops.Loop;
import org.usfirst.frc.team1671.loops.Looper;
import org.usfirst.frc.team1671.robot.RobotMap;
import org.usfirst.frc.team1671.states.IntakeState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import drivers.VictorSPXFactory;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Intake extends Subsystem {
	private static Intake instance = null;
	
	public static Intake getInstance() {
		if(instance == null) {
			instance = new Intake();
		}
		return instance;
	}
	
	
	
	//Hardware
	private VictorSPX leftVictor, rightVictor;
	private Solenoid upperPiston, lowerPiston;
	private AnalogInput intakeBanner;
	
	//Control
	private IntakeState wantedState;
	private IntakeState current;
	
	//Loop
	private Loop loop = new Loop() {

		@Override
		public void onStart(double timestamp) {
			stop();	
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized(Intake.this) {
				handleState();
			}
		}

		@Override
		public void onStop(double timestamp) {
			stop();
		}
		
	};
	
	private Intake() {
		leftVictor = VictorSPXFactory.createDefaultVictor(RobotMap.INTAKE_MOTOR_LEFT);
		leftVictor.set(ControlMode.PercentOutput, 0.0);
		leftVictor.setInverted(false);
		leftVictor.configVoltageCompSaturation(12.0, RobotMap.kLongCANTimeoutMs);
		leftVictor.enableVoltageCompensation(true);
		
		rightVictor = VictorSPXFactory.createDefaultVictor(RobotMap.INTAKE_MOTOR_RIGHT);
		rightVictor.set(ControlMode.PercentOutput, 0.0);
		rightVictor.setInverted(false);
		rightVictor.configVoltageCompSaturation(12.0, RobotMap.kLongCANTimeoutMs);
		rightVictor.enableVoltageCompensation(true);
		
		upperPiston = new Solenoid(RobotMap.INTAKE_PISTON_TOP);
		lowerPiston = new Solenoid(RobotMap.INTAKE_PISTON_BOTTOM);
		
		intakeBanner = new AnalogInput(RobotMap.INTAKE_BANNER);
		
		current = new IntakeState();
	}
	
	private void handleState() {
		runIntake(wantedState.leftMotor, wantedState.rightMotor);
		current.setPower(wantedState.leftMotor);
		
		current.cubeDetected = getBannerSensor();
		
		switch(wantedState.jawState) {
		case OPEN:
			open();
			return;
		case NEUTRAL:	
			neutral();
			return;
		case CLOSED:
			close();
			return;
		default:
			System.out.println("Unexpected Jaw state: " + wantedState.jawState);
			break;
		}
		current.jawState = wantedState.jawState;
	}
	
	
	private void open() {
		upperPiston.set(false);
		lowerPiston.set(true);
	}
	
	private void close() {
		upperPiston.set(true);
		lowerPiston.set(false);
	}
	
	private void neutral() {
		upperPiston.set(false);
		lowerPiston.set(false);
	}
	
	//positive is in, negative is out
	private void runIntake(double left, double right) {
		leftVictor.set(ControlMode.PercentOutput, left);
		rightVictor.set(ControlMode.PercentOutput, right);
	}
	
	private boolean getBannerSensor() {
		return intakeBanner.getVoltage() > 4;
	}
	
	public IntakeState.JawState getJawState() {
		return current.jawState;
	}
	
	public synchronized IntakeState getCurrentState() {
		return current;
	}
	
	public synchronized void setWantedState(IntakeState wanted) {
		wantedState = wanted;
	}
	
	public synchronized IntakeState getWantedState() {
		return wantedState;
	}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putBoolean("Banner Sensor", getBannerSensor());
	}

	@Override
	public synchronized void stop() {
		runIntake(0.0, 0.0);
		close();
		wantedState = new IntakeState();
	}

	@Override
	public void zeroSensors() {
	}

	@Override
	public void registerEnabledLoops(Looper enabledLooper) {
		enabledLooper.register(loop);
	}

}
