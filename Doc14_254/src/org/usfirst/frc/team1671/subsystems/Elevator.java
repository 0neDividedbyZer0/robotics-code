package org.usfirst.frc.team1671.subsystems;

import org.usfirst.frc.team1671.loops.Loop;
import org.usfirst.frc.team1671.loops.Looper;
import control.ProfileConstraints;
import control.ProfileController;
import control.ProfilePoint;
import org.usfirst.frc.team1671.robot.RobotMap;
import control.TrapezoidalProfile;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import drivers.VictorSPXFactory;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import util.Util;

//NOTE: ONLY THE RIGHT ENCODER IS BEING USED RIGHT NOW
/**
 * 
 * @author Maverick Zhang
 *
 */
public class Elevator extends Subsystem {
	
	private static Elevator instance = null;
	
	public static Elevator getInstance() {
		if(instance == null) {
			instance = new Elevator();
		}
		return instance;
	}
	
	public enum ControlMethod {
		OPEN_LOOP,
		PROFILED
	}
	
	//Hardware
	private VictorSPX leftTop, leftBottom, rightTop, rightBottom;
	//private DoubleSolenoid elevShifter; //But shifter is broken :/
	private Encoder leftEnc, rightEnc;
	
	//Profiles
	private ProfilePoint goal;
	private ProfilePoint updatingGoal;
	private ProfileController controller;
	private ProfileConstraints constraints;
	
	//Control Method
	private ControlMethod method;
	
	//Hardware State
	private boolean isBrakeMode;
	
	private Loop loop = new Loop() {
		@Override
		public void onStart(double timestamp) {
			stop();
		}
		
		@Override
		public void onLoop(double timestamp) {
			synchronized(Elevator.this) {
				switch(method) {
				case OPEN_LOOP:
					return;
				case PROFILED:
					handleProfiled();
					return;
				default:
					System.out.println("Unexpected elevator control method: " + method);
					break;
				}
			}
		}
		
		@Override
		public void onStop(double timestamp) {
			stop();
		}
	};
	
	private Elevator() {
		leftTop = VictorSPXFactory.createDefaultVictor(RobotMap.LEFT_TOP_ELEVATOR);
		leftTop.enableVoltageCompensation(true);
		
		leftBottom = VictorSPXFactory.createPermanentSlaveVictor(RobotMap.LEFT_BOTTOM_ELEVATOR, RobotMap.LEFT_TOP_ELEVATOR);
		
		rightTop = VictorSPXFactory.createPermanentSlaveVictor(RobotMap.RIGHT_TOP_ELEVATOR, RobotMap.LEFT_TOP_ELEVATOR);
		rightBottom = VictorSPXFactory.createPermanentSlaveVictor(RobotMap.RIGHT_BOTTOM_ELEVATOR, RobotMap.LEFT_TOP_ELEVATOR);
	
		//elevShifter = new DoubleSolenoid(RobotMap.ELEVATOR_SHIFTER_FORWARD, RobotMap.ELEVATOR_SHIFTER_REVERSE);
		
		leftEnc = new Encoder(RobotMap.LEFT_ELEVATOR_ENC_1, RobotMap.LEFT_ELEVATOR_ENC_2, false);
		rightEnc = new Encoder(RobotMap.RIGHT_ELEVATOR_ENC_1, RobotMap.RIGHT_ELEVATOR_ENC_2, true);
		
		leftEnc.setDistancePerPulse(RobotMap.ELEV_ENC_RATE);
		rightEnc.setDistancePerPulse(RobotMap.ELEV_ENC_RATE);
		
		constraints = new ProfileConstraints(0.95 * RobotMap.MAX_ELEV_VELOCITY, RobotMap.MAX_ELEV_ACCEL);
		goal = new ProfilePoint(0.0, 0.0);
		updatingGoal = new ProfilePoint(0.0, 0.0);
		controller = new ProfileController(RobotMap.ELEV_KP, RobotMap.ELEV_KI, RobotMap.ELEV_KD, RobotMap.ELEV_KV);
	}
	
	//if -voltage is positive go up
	private void setMotors(double voltage) {
		leftTop.set(ControlMode.PercentOutput, -voltage);
	}
	
	//Expects inverted voltages :/ 
	public synchronized void setOpenLoop(double voltage) {
		if(method != ControlMethod.OPEN_LOOP) {
			method = ControlMethod.OPEN_LOOP;
			setBrakeMode(false);
		}
		setMotors(makeSafe(voltage));
	}
	
	public synchronized void setProfiled() {
		if(method != ControlMethod.PROFILED) {
			method = ControlMethod.PROFILED;
			setBrakeMode(true);
			updatingGoal = new ProfilePoint(rightEnc.getDistance(), rightEnc.getRate());
		}
	}
	
	public synchronized void setBrakeMode(boolean on) {
		if(isBrakeMode != on) {
			isBrakeMode = on;
			if(isBrakeMode == true) {
				leftTop.setNeutralMode(NeutralMode.Brake);
				leftBottom.setNeutralMode(NeutralMode.Brake);
				
				rightTop.setNeutralMode(NeutralMode.Brake);
				rightBottom.setNeutralMode(NeutralMode.Brake);
			} else {
				leftTop.setNeutralMode(NeutralMode.Coast);
				leftBottom.setNeutralMode(NeutralMode.Coast);
				
				rightTop.setNeutralMode(NeutralMode.Coast);
				rightBottom.setNeutralMode(NeutralMode.Coast);
			}
		}
	}
	
	public synchronized void setGoal(double height) {
		if(height > RobotMap.MAX_ELEV_HEIGHT) {
			height = RobotMap.MAX_ELEV_HEIGHT;
		}
		if(height < RobotMap.MIN_ELEV_HEIGHT) {
			height = RobotMap.MIN_ELEV_HEIGHT;
		}
		goal = new ProfilePoint(height, 0.0);
	}
	
	private void handleProfiled() {
		updateProfile();
		double voltage = controller.calculate(updatingGoal, rightEnc.getDistance(), RobotMap.kLooperDt);
		setMotors(-voltage);
	}
	
	private ProfilePoint updateProfile() {
		TrapezoidalProfile profile = new TrapezoidalProfile(constraints, goal, updatingGoal);
		updatingGoal = profile.calculate(RobotMap.kLooperDt);
		return updatingGoal;
	}
	
	private void resetEncoders() {
		leftEnc.reset();
		rightEnc.reset();
	}
	
	public synchronized void setPID(double kP, double kI, double kD, double kV) {
		controller.setPID(kP, kI, kD, kV);
	}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("left elevator", leftEnc.getDistance());
		SmartDashboard.putNumber("right elevator", rightEnc.getDistance());
	}

	@Override
	public synchronized void stop() {
		setOpenLoop(0.0);
	}

	@Override
	public void zeroSensors() {
		resetEncoders();
	}
	
	public double getHeight() {
		return rightEnc.getDistance();
	}
	
	private double makeSafe(double voltage) {
		if(voltage < 0.0 && Util.epsilonEquals(rightEnc.getDistance(), RobotMap.MAX_ELEV_HEIGHT, 3.0)) {
			return 0.0;
		} else if(voltage > 0.0 && Util.epsilonEquals(rightEnc.getDistance(), RobotMap.MIN_ELEV_HEIGHT, 2.0)) {
			return 0.0;
		} else {
			return voltage;
		}
	}

	@Override
	public void registerEnabledLoops(Looper in) {
		in.register(loop);
	}

}
