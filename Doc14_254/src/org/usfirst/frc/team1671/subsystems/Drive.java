package org.usfirst.frc.team1671.subsystems;

import org.usfirst.frc.team1671.loops.Loop;
import org.usfirst.frc.team1671.loops.Looper;
import org.usfirst.frc.team1671.robot.DriveSignal;
import org.usfirst.frc.team1671.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import control.PathController;
import control.ProfileConstraints;
import control.ProfileController;
import control.ProfilePoint;
import control.TrapezoidalProfile;
import drivers.TalonSRXFactory;
import drivers.VictorSPXFactory;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import trajectory_lib.AutoTrajectory;
import trajectory_lib.TrajectoryFollower;

/**
 * 
 * @author Maverick Zhang
 *
 */
public class Drive extends Subsystem {
	
	private static Drive instance = new Drive();
	
	//Singleton
	public static Drive getInstance() {
		return instance;
	}
	
	public enum DriveState {
		OPEN_LOOP,
		SIMPLE_DRIVE,
		PATH_FOLLOWING,
		TURN_TO_HEADING
	}
	
	//Control State
	private DriveState state;
	
	//Hardware 
	private final TalonSRX leftRear, rightRear;
	private final VictorSPX leftMid, leftFront, rightMid, rightFront;
	private final DoubleSolenoid shifter;
	private final Encoder leftEnc, rightEnc;
	private final PigeonIMU imu;
	
	//Controllers
	private ProfilePoint goal; //For SimpleDrive (1d motion profiled)
	private ProfilePoint updatingGoal; //For SimpleDrive
	private ProfileController controller; //For SimpleDrive
	private ProfileConstraints constraints; //For SimpleDrive
	private double simpleDriveStartPos = 0.0; //For SimpleDrive.
	private ProfileController headingController; //For TurnToHeading
	private TrajectoryFollower follower; //For Path Following
	private double initPosLeft = 0.0, initPosRight = 0.0; //For Path Following
	private boolean reverse = false;
	
	//Hardware States
	private boolean isHighGear;
	private boolean isBrakeMode;
	private double offsetAngle = 0.0;
	
	private final Loop loop = new Loop() {
		@Override
		public void onStart(double timestamp) {
			synchronized (Drive.this) {
				setOpenLoop(DriveSignal.NEUTRAL);
				setBrakeMode(false);
			}
		}
		
		@Override
		public void onLoop(double timestamp) {
			synchronized (Drive.this) {
				switch(state) {
				case OPEN_LOOP:
					return;
				case SIMPLE_DRIVE:
					handleSimpleDrive();
					return;
				case PATH_FOLLOWING:
					handlePathFollowing();
					return;
				case TURN_TO_HEADING:
					handleTurnToHeading();
					return;
				default:
					System.out.println("Unexpected drive control state: " + state);
					break;
				}
			}
		}
		
		@Override
		public void onStop(double timestamp) {
			stop();
		}
	};
	
	private Drive() {
		leftFront = VictorSPXFactory.createDefaultVictor(RobotMap.LEFT_FRONT_VICTOR_PORT);
		leftMid = VictorSPXFactory.createPermanentSlaveVictor(RobotMap.LEFT_MID_VICTOR_PORT, RobotMap.LEFT_FRONT_VICTOR_PORT);
		leftRear = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.LEFT_REAR_TALON_PORT, leftFront);
		
		rightFront = VictorSPXFactory.createDefaultVictor(RobotMap.RIGHT_FRONT_VICTOR_PORT);
		rightMid = VictorSPXFactory.createPermanentSlaveVictor(RobotMap.RIGHT_MID_VICTOR_PORT, RobotMap.RIGHT_FRONT_VICTOR_PORT);
		rightRear = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.RIGHT_REAR_TALON_PORT, rightFront);
		
		leftFront.enableVoltageCompensation(true);
		rightFront.enableVoltageCompensation(true);
		
		shifter = new DoubleSolenoid(RobotMap.DRIVE_SHIFTER_FORWARD, RobotMap.DRIVE_SHIFTER_REVERSE);
		
		leftEnc = new Encoder(RobotMap.LEFT_ENCODER_1, RobotMap.LEFT_ENCODER_2, RobotMap.DRIVE_LEFT_ENC_DIRECTION); //Should be true for Doc
		rightEnc = new Encoder(RobotMap.RIGHT_ENCODER_1, RobotMap.RIGHT_ENCODER_2, RobotMap.DRIVE_RIGHT_ENC_DIRECTION); //Should be false for Doc
		
		leftEnc.setDistancePerPulse(RobotMap.DIAMETER * Math.PI / 350.0);
		rightEnc.setDistancePerPulse(RobotMap.DIAMETER * Math.PI / 353.0);
		
		imu = new PigeonIMU(RobotMap.IMU);
		
		isHighGear = false;
		
		isBrakeMode = true;
		
		constraints = new ProfileConstraints(0.5 * RobotMap.FOLLOWER_MAX_V, RobotMap.FOLLOWER_MAX_A);
		goal = new ProfilePoint(0.0, 0.0);
		updatingGoal = new ProfilePoint(0.0, 0.0);
		controller = new ProfileController(RobotMap.SIMPLE_KP, RobotMap.SIMPLE_KI, RobotMap.SIMPLE_KD, RobotMap.SIMPLE_KV);
		
		headingController = new ProfileController(-RobotMap.IN_PLACE_KP, -RobotMap.IN_PLACE_KI, RobotMap.IN_PLACE_KD);
		
		
		follower = new TrajectoryFollower(RobotMap.PATH_FOLLOWER_KP, RobotMap.PATH_FOLLOWER_KI, RobotMap.PATH_FOLLOWER_KD, 
				RobotMap.PATH_FOLLOWER_KV, RobotMap.PATH_FOLLOWER_KA, RobotMap.PATH_FOLLOWER_KW);
		
	}
	
	@Override
	public void registerEnabledLoops(Looper in) {
		in.register(loop);
	}
	
	private void setMotors(DriveSignal signal) {
		//leftFront takes in negatives and goes forward
		leftFront.set(ControlMode.PercentOutput, -signal.getLeft());
		//rightFront takes in positives and go forward
		rightFront.set(ControlMode.PercentOutput, signal.getRight());
	}
	
	/**
	 * Sets TurnInPlace angle absolutely, based on the FieldElements orientation (0 degrees is in the direction <1,0>, 
	 * and + angle is counterclockwise). In Degrees
	 * @param angle
	 */
	public synchronized void setAngleGoalAbs(double angle) {
		headingController.setSetpoint(angle);
	}
	
	/**
	 * Sets TurnInPlace angle relatively to your current angle, based on FieldElements orientation
	 * @param angle
	 */
	public synchronized void setAngleGoalRel(double angle) {
		double angleGoal = getAngle() + angle;
		headingController.setSetpoint(angleGoal);
	}
	
	public synchronized double getAngleGoal() {
		return headingController.getSetpoint();
	}
	
	public double getVelocity() {
		return (leftEnc.getRate() + rightEnc.getRate()) / 2.0;
	}
	
	public double getLeftEncRate() {
		return leftEnc.getRate();
	}
	
	public double getRightEncRate() {
		return rightEnc.getRate();
	}
	
	private void handleTurnToHeading() {
		double voltage = headingController.calculate(getAngle(), RobotMap.kLooperDt);
		DriveSignal signal = new DriveSignal(voltage, -voltage, true);
		setMotors(signal);
	}
	
	public synchronized void setOpenLoop(DriveSignal signal) {
		if(state != DriveState.OPEN_LOOP) {
			state = DriveState.OPEN_LOOP;
			setBrakeMode(false);
		}
		setMotors(signal);
	}
	
	public synchronized void setState(DriveState wantedState) {
		if(state != wantedState) {
			state = wantedState;
		}
	}
	
	public synchronized double getSimpleGoal() {
		return goal.getPos();
	}
	
	//Inches, maybe change to feet?
	public synchronized void setSimpleGoal(double distance) {
		goal = new ProfilePoint(distance, 0.0);
		simpleDriveStartPos = rightEnc.getDistance();
		updatingGoal = new ProfilePoint(0.0, 0.0);
	}
	
	private void handleSimpleDrive() {
		updateProfile();
		double voltage = controller.calculate(updatingGoal, inToFt(rightEnc.getDistance() - simpleDriveStartPos), RobotMap.kLooperDt);
		DriveSignal signal = new DriveSignal(voltage, voltage);
		setMotors(signal);
		System.out.println(leftFront.getMotorOutputVoltage() + " + " +rightFront.getMotorOutputVoltage());
	}
	
	private ProfilePoint updateProfile() {
		TrapezoidalProfile profile = new TrapezoidalProfile(constraints, goal, updatingGoal);
		updatingGoal = profile.calculate(RobotMap.kLooperDt);
		return updatingGoal;
	}
	
	public boolean isHighGear() {
		return isHighGear;
	}
	
	public synchronized void setHighGear(boolean wantHighGear) {
		if(wantHighGear != isHighGear) {
			isHighGear = wantHighGear;
			if(isHighGear == true) {
				shifter.set(Value.kForward);
			} else if(isHighGear == false) {
				shifter.set(Value.kReverse);
			}
		}
	}
	
	public boolean isBrakeMode() {
		return isBrakeMode;
	}
	
	public synchronized void setBrakeMode(boolean on) {
		if(isBrakeMode != on) {
			isBrakeMode = on;
			if(isBrakeMode == true) {
				leftFront.setNeutralMode(NeutralMode.Brake);
				leftMid.setNeutralMode(NeutralMode.Brake);
				leftRear.setNeutralMode(NeutralMode.Brake);
				
				rightFront.setNeutralMode(NeutralMode.Brake);
				rightMid.setNeutralMode(NeutralMode.Brake);
				rightRear.setNeutralMode(NeutralMode.Brake);
			} else if(isBrakeMode == false) {
				leftFront.setNeutralMode(NeutralMode.Coast);
				leftMid.setNeutralMode(NeutralMode.Coast);
				leftRear.setNeutralMode(NeutralMode.Coast);
				
				rightFront.setNeutralMode(NeutralMode.Coast);
				rightMid.setNeutralMode(NeutralMode.Coast);
				rightRear.setNeutralMode(NeutralMode.Coast);
			}
		}
	}
	
	@Override
	public synchronized void stop() {
		setOpenLoop(DriveSignal.NEUTRAL);
	}


	@Override
	public void outputToSmartDashboard() {
		final double leftSpeed = leftEnc.getRate();
		final double rightSpeed = rightEnc.getRate();
		SmartDashboard.putNumber("left distance", leftEnc.getDistance());
		SmartDashboard.putNumber("right distance", rightEnc.getDistance());
		
		SmartDashboard.putNumber("left voltage (V)", leftFront.getMotorOutputVoltage());
		SmartDashboard.putNumber("right voltage (V)", rightFront.getMotorOutputVoltage());
		SmartDashboard.putNumber("left speed (ips)", leftSpeed);
		SmartDashboard.putNumber("right speed (ips)", rightSpeed);
		
		SmartDashboard.putNumber("gyro vel", getAngularSpeed());
		SmartDashboard.putNumber("gyro pos", getAngle());
	}
	
	public synchronized void resetGyro() {
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		offsetAngle = imu.getFusedHeading(fusionStatus);
	}
	
	//Degrees per second
	public double getAngularSpeed() {
		double[] rpy = new double[3];
		imu.getRawGyro(rpy);
		return rpy[2];
	}
	
	//Degrees
	public double getAngle() {
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		return imu.getFusedHeading(fusionStatus) - offsetAngle;
	}
	
	public double getRightEnc() {
		return rightEnc.getDistance();
	}
	
	public double getLeftEnc() {
		return leftEnc.getDistance();
	}
	
	public double inToFt(double inches) {
		return inches / 12.0;
	}
	
	public double ftToIn(double feet) {
		return 12.0 * feet;
	}
	
	public synchronized void resetEncoders() {
		leftEnc.reset();
		rightEnc.reset();
	}
	
	public synchronized void setTrajectory(AutoTrajectory auto, boolean reverse) {
		follower.setTrajectory(auto);
		initPosLeft = inToFt(leftEnc.getDistance());
		initPosRight = inToFt(rightEnc.getDistance());
		follower.start(initPosLeft, initPosRight, reverse);
	}
	
	private void handlePathFollowing() {
		DriveSignal signal = updatePathFollowing(inToFt(leftEnc.getDistance()), inToFt(leftEnc.getRate()), 
				inToFt(rightEnc.getDistance()), inToFt(rightEnc.getRate()), getAngle());
		System.out.println(signal.getLeft());
		setMotors(signal);
	}
	
	private DriveSignal updatePathFollowing(double leftEncoder, double leftSpeed, double rightEncoder, double rightSpeed, double gyro) {
		return process(follower.calculate(leftEncoder, leftSpeed, rightEncoder, rightSpeed, gyro, RobotMap.kLooperDt));
	}
	
	private DriveSignal process(DriveSignal in) {
		/*double leftSignal = 0.875 * in.getLeft();
		double rightSignal = 0.875 * in.getRight();
		if(leftSignal > 0 ) {
			leftSignal += 0.127;
		} else if(leftSignal < 0) {
			leftSignal -= 0.127;
		} else {
			leftSignal = 0.0;
		}
		
		if(rightSignal > 0 ) {
			rightSignal += 0.127;
		} else if(rightSignal < 0) {
			rightSignal -= 0.127;
		} else {
			rightSignal = 0.0;
		}*/
		
		return new DriveSignal(in.getLeft(), in.getRight(), in.getBrakeMode());
	}
	
	public synchronized boolean pathFinished() {
		return follower.isFinished();
	}

	@Override
	public void zeroSensors() {
		resetEncoders();
		resetGyro();
	}

}
