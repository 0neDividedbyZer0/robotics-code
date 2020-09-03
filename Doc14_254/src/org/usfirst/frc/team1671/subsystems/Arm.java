package org.usfirst.frc.team1671.subsystems;

import org.ojalgo.matrix.PrimitiveMatrix;
import org.usfirst.frc.team1671.loops.Loop;
import org.usfirst.frc.team1671.loops.Looper;
import org.usfirst.frc.team1671.robot.Controller;
import control.ProfileConstraints;
import control.ProfileController;
import control.ProfilePoint;
import org.usfirst.frc.team1671.robot.RobotMap;
import control.TrapezoidalProfile;
import org.usfirst.frc.team1671.subsystems.Drive.DriveState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import control.StateSpaceController;
import control.StateSpaceObserver;
import control.StateSpacePlant;
import drivers.VictorSPXFactory;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem {
	private static Arm instance = null;
	
	//Singleton
	public static Arm getInstance() {
		if(instance == null) {
			instance = new Arm();
		}
		return instance;
	}

    //Hardware
	private VictorSPX armMotor;
	private Encoder armEnc;
	
	//Hardware State
	private boolean isBrakeMode;
	private static double tolerance = 1.0;
	
	//Controllers
	private ProfileController controller;
	private ProfileConstraints constraints;
	private ProfilePoint armGoal; //This is your end desired goal for the trapezoidal motion profile
	private ProfilePoint updatingGoal; //This is essentially your current goal to get to. This will be updated
	
	//Experimental, Don't touch
	private StateSpaceController stateSpaceController = new StateSpaceController(1,2,1);
	private StateSpacePlant plant = new StateSpacePlant(1,2,1);
	private StateSpaceObserver observer = new StateSpaceObserver(1,2,1);
	
	//Control Mode
	private ControlMethod method;

	public enum ControlMethod {
		OPEN_LOOP,
		PROFILED
	}
	
	//Internal loop of the robot, required for functionality.
	private Loop loop = new Loop() {
		@Override
		public void onStart(double timestamp) {
			stop();
		}
		
		@Override
		public void onLoop(double timestamp) {
			synchronized (Arm.this) {
				switch(method) {
				case OPEN_LOOP:
					return;
				case PROFILED:
					handleProfiled();
					return;
				default:
					System.out.println("Unexpected arm control method: " + method);
					break;
				}
			}
		}
		
		@Override
		public void onStop(double timestamp) {
			stop();
		}
	};
	
	private Arm() {
		armMotor = VictorSPXFactory.createDefaultVictor(RobotMap.ARM_MOTOR);
		armMotor.setInverted(false);
		armMotor.setNeutralMode(NeutralMode.Brake);
		armMotor.enableVoltageCompensation(true);
		armMotor.set(ControlMode.PercentOutput, 0.0);
		
		armEnc = new Encoder(RobotMap.ARM_ENCODER_1, RobotMap.ARM_ENCODER_2, true);
		armEnc.setDistancePerPulse(RobotMap.ARM_ENCODER_RATE);
		
		constraints = new ProfileConstraints(0.8 * RobotMap.ARM_MAX_SPEED, RobotMap.ARM_MAX_ACCELERATION);
		
		controller = new ProfileController(RobotMap.ARM_KP, RobotMap.ARM_KI, RobotMap.ARM_KD, RobotMap.ARM_KV);
		
		armGoal = new ProfilePoint(0.0, 0.0);
		
		updatingGoal = new ProfilePoint(0.0, 0.0);
		
		/*stateSpaceController.setA(RobotMap.A);
		stateSpaceController.setK(RobotMap.K);
		stateSpaceController.setKff(RobotMap.Kff);*/
		stateSpaceController.setU_max(RobotMap.ARM_U_MAX);
		stateSpaceController.setU_min(RobotMap.ARM_U_MIN);
		
		/*plant.setA(RobotMap.A);
		plant.setB(RobotMap.B);
		plant.setC(RobotMap.C);
		plant.setD(RobotMap.D);
		
		observer.setL(RobotMap.L);*/
		observer.setPlant(plant);
	}
	
	public synchronized void setProfiled() {
		if(method != ControlMethod.PROFILED) {
			method = ControlMethod.PROFILED;
			setBrakeMode(true);
		}
	}
	
	//positive goes down.
	private void setArm(double voltage) {
		//armMotor.set(ControlMode.PercentOutput, voltage);
	}
	//This is used to update the profile and set the arm
	private void handleProfiled() {
		updateProfile();
		double voltage = controller.calculate(updatingGoal, armEnc.getDistance(), RobotMap.kLooperDt);
		if(Math.abs(getAngle() - armGoal.getPos()) < tolerance) {
			voltage = 0.0;
		}
		
		setArm(voltage);
	}
	
	//This changes the armGoal (your desired end goal)
	public synchronized void setGoal(double armAngle) {
		if(armAngle > RobotMap.ARM_MAX_ANGLE) {
			armAngle = RobotMap.ARM_MAX_ANGLE;
		} else if(armAngle < RobotMap.ARM_MIN_ANGLE) {
			armAngle = RobotMap.ARM_MIN_ANGLE;
		}
		armGoal = new ProfilePoint(armAngle, 0.0);
	}
	
	//This will change the updatingGoal to your next desired goal.
	//The updatingGoal is returned
	private ProfilePoint updateProfile() {
		TrapezoidalProfile profile = new TrapezoidalProfile(constraints, armGoal, updatingGoal);
		updatingGoal = profile.calculate(RobotMap.kLooperDt);
		return updatingGoal;
	}
	
	//Not in use really, but will find the time in a profile to get to an angle, from your current angle.
	public synchronized double timeUntil(double angle, double finalAngle) {
		if(armGoal.getPos() > angle) {
			return 0.0;
		}
		TrapezoidalProfile profile = new TrapezoidalProfile(constraints, new ProfilePoint(finalAngle, 0.0), updatingGoal);
		return profile.timeUntil(angle);
	}
	
	//For manual arm control, but effectively useless now since manual arm control has since
	//been removed.
	public synchronized void setOpenLoop(double voltage) {
		if(method != ControlMethod.OPEN_LOOP) {
			method = ControlMethod.OPEN_LOOP;
			setBrakeMode(false);
		}
		setArm(voltage);
	}
	
	//Just don't
	public synchronized void setPID(double kP, double kI, double kD, double kV) {
		controller.setPID(kP, kI, kD, kV);
	}
	
	public synchronized void setBrakeMode(boolean on) {
		if(isBrakeMode != on) {
			isBrakeMode = on;
			if(isBrakeMode == true) {
				armMotor.setNeutralMode(NeutralMode.Brake);
			} else {
				armMotor.setNeutralMode(NeutralMode.Coast);
			}
		}
	}
	
	public synchronized void resetEncoders() {
		armEnc.reset();
	}
	
	public synchronized double getAngle() {
		return armEnc.getDistance();
	}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("arm", getAngle());
	}

	@Override
	public synchronized void stop() {
		setOpenLoop(0.0);
	}

	@Override
	public void zeroSensors() {
		resetEncoders();
	}

	@Override
	public void registerEnabledLoops(Looper in) {
		in.register(loop);
	}
	
	
}
