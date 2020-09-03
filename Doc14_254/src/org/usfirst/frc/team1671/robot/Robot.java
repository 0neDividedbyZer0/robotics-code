/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1671.robot;

import java.util.Arrays;

import org.usfirst.frc.team1671.auto.AutoModeExecuter;
import org.usfirst.frc.team1671.auto.modes.CrossAutoLineMode;
import org.usfirst.frc.team1671.auto.modes.DoNothingMode;
import org.usfirst.frc.team1671.auto.modes.PathFollowingScaleMode;
import org.usfirst.frc.team1671.auto.modes.PathFollowingSwitchMode;
import org.usfirst.frc.team1671.auto.modes.TestMode;
import org.usfirst.frc.team1671.loops.CrashTracker;
import org.usfirst.frc.team1671.loops.Looper;
import org.usfirst.frc.team1671.states.IntakeState;
import org.usfirst.frc.team1671.subsystems.Arm;
import org.usfirst.frc.team1671.subsystems.ConnectionMonitor;
import org.usfirst.frc.team1671.subsystems.Drive;
import org.usfirst.frc.team1671.subsystems.Elevator;
import org.usfirst.frc.team1671.subsystems.Intake;
import org.usfirst.frc.team1671.subsystems.LEDs;
import org.usfirst.frc.team1671.subsystems.LEDs.LEDState;
import org.usfirst.frc.team1671.subsystems.Superstructure;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.modifiers.TankModifier;
import util.ArcadeDriveHelper;
import util.PathfinderGenerator;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	//First you need every subsystem to be created
	//Every subsystem is a singleton, meaning only one version of it exists
	//Calling each subsystem from here is the same across all of the robot classes
	private Drive drive = Drive.getInstance();
	private Intake intake = Intake.getInstance();
	private Arm arm = Arm.getInstance();
	private Elevator elevator = Elevator.getInstance();
	private LEDs leds = LEDs.getInstance();
	private Superstructure superstructure = Superstructure.getInstance();
	private AutoModeExecuter autoModeExecuter = new AutoModeExecuter();
	private TankModifier modifier;
	
	//A Subsystem Manager for Convenience
	//Each subsystem has an internal looper. Adding or removing a subsystem
	//From this list will effectively add or remove functionality from the subsystem.
	private final SubsystemManager subsystemManager = new SubsystemManager(
			Arrays.asList(Drive.getInstance(), ConnectionMonitor.getInstance(), Intake.getInstance(), 
					Arm.getInstance(), Elevator.getInstance(), Superstructure.getInstance(),LEDs.getInstance()));
	
	//Some 254 stuff to run faster loops at 200 hz, or 0.005 seconds
	Looper internalLooper = new Looper();
	
	//Xbox Controllers
	private static Controller base = new Controller(0);
	private static Controller co = new Controller(1);
	
	private static AxisGreater shiftTrigger = new AxisGreater(base, 3, 0.3);
	
	//This does calculations for ArcadeDrive
	private ArcadeDriveHelper arcadeDriveHelper = new ArcadeDriveHelper();
	
	boolean wantIntake = false;
	boolean holdIntake = false;
	
	//Note:Sensors are zeroed upon RobotInit, AutonomousInit.
	
	public enum AutoMode {
		DO_NOTHING, // start
		CROSS_AUTO_LINE, //y
		SWITCH, //a
		LEFT_START, //x
		RIGHT_START, //b
		TEST // left bumper
	}
	
	private AutoMode autoMode = AutoMode.DO_NOTHING;
	
	public Robot() {
		CrashTracker.logRobotConstruction();
	}
	
	public void zeroSensors() {
		subsystemManager.zeroSensors();
		drive.zeroSensors();
	}
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		try {
			subsystemManager.registerEnabledLoops(internalLooper);
			leds.setState(LEDState.INIT);
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(640, 480);
			camera.setFPS(20);
			
			/*modifier = PathfinderGenerator.generateTrajectory(PathElements.startToSwitchLeft);
			
			PathfinderGenerator.save("/home/lvuser/paths/lSwitchSL1", modifier.getLeftTrajectory());
			PathfinderGenerator.save("/home/lvuser/paths/lSwitchSR1", modifier.getRightTrajectory());
			
			modifier = PathfinderGenerator.generateTrajectory(PathElements.startToSwitchRight);
			
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSL1", modifier.getLeftTrajectory());
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSR1", modifier.getRightTrajectory());*/
			
			/*modifier = PathfinderGenerator.generateTrajectory(PathElements.switchRightToShiftedCenter);
			
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSL2", modifier.getLeftTrajectory());
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSR2", modifier.getRightTrajectory());
			
			modifier = PathfinderGenerator.generateTrajectory(PathElements.shiftedCenterToSwitchCube1);
			
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSL3", modifier.getLeftTrajectory());
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSR3", modifier.getRightTrajectory());
			
			modifier = PathfinderGenerator.generateTrajectory(PathElements.switchCube1ToShiftedCenter);
			
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSL4", modifier.getLeftTrajectory());
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSR4", modifier.getRightTrajectory());
			
			modifier = PathfinderGenerator.generateTrajectory(PathElements.shiftedCenterToSwitchRight);
			
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSL5", modifier.getLeftTrajectory());
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSR5", modifier.getRightTrajectory());
			
			modifier = PathfinderGenerator.generateTrajectory(PathElements.switchRightToShiftedCenter2);
			
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSL6", modifier.getLeftTrajectory());
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSR6", modifier.getRightTrajectory());
			
			modifier = PathfinderGenerator.generateTrajectory(PathElements.shiftedCenterToSwitchCube2);
			
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSL7", modifier.getLeftTrajectory());
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSR7", modifier.getRightTrajectory());
			
			modifier = PathfinderGenerator.generateTrajectory(PathElements.switchCube2ToShiftedCenter);
			
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSL8", modifier.getLeftTrajectory());
			PathfinderGenerator.save("/home/lvuser/paths/rSwitchSR8", modifier.getRightTrajectory());*/
			
			//modifier = PathfinderGenerator.generateTrajectory(PathElements.shiftedCenterToSwitchRight2);
			
			//PathfinderGenerator.save("/home/lvuser/paths/rSwitchSL9", modifier.getLeftTrajectory());
			//PathfinderGenerator.save("/home/lvuser/paths/rSwitchSR9", modifier.getRightTrajectory());
			
		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
		zeroSensors();
	}

	
	@Override
	public void autonomousInit() {
		try {
			CrashTracker.logAutoInit();
			
			String gameData = DriverStation.getInstance().getGameSpecificMessage();
			
			System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());
			
			if (autoModeExecuter != null) {
                autoModeExecuter.stop();
            }
			
			zeroSensors();
			
			autoModeExecuter = null;
			
			drive.setHighGear(true);
			drive.setBrakeMode(true);
			
			//Setting this to an auto will cause autonomous to operate
			internalLooper.start();
			autoModeExecuter = new AutoModeExecuter();
			switch(autoMode) {
				case DO_NOTHING:
					autoModeExecuter.setAutoMode(new DoNothingMode());
					break;
				case CROSS_AUTO_LINE:
					autoModeExecuter.setAutoMode(new CrossAutoLineMode());
					break;
				case SWITCH:
					autoModeExecuter.setAutoMode(new PathFollowingSwitchMode(gameData));
					break;
				case LEFT_START:
					autoModeExecuter.setAutoMode(new PathFollowingScaleMode(false, gameData));
					break;
				case RIGHT_START:
					autoModeExecuter.setAutoMode(new PathFollowingScaleMode(true, gameData));
					break;
				case TEST:
					autoModeExecuter.setAutoMode(new TestMode());
					break;
				default:
					System.out.println("Unexpected Auto mode, running Do Nothing");
					autoModeExecuter.setAutoMode(new DoNothingMode());
			}
			autoModeExecuter.start();
		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		allPeriodic();
	}
	
	@Override
	public void teleopInit() {
		try {
			CrashTracker.logTeleopInit();
			if(autoModeExecuter != null) {
				autoModeExecuter.stop();
			}
			
			internalLooper.start();
			drive.setOpenLoop(DriveSignal.NEUTRAL);
			drive.setBrakeMode(false);
			drive.setHighGear(true);
			
		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		try {
			double timestamp = Timer.getFPGATimestamp();
			
			//Limiting logic
			double currentHeight = elevator.getHeight();
			
			double speedLimit = 1.0 - (0.2/ RobotMap.MAX_ELEV_HEIGHT) * currentHeight;
			double turnLimit = 1.0 - (0.5 / RobotMap.MAX_ELEV_HEIGHT) * currentHeight;
			
			double throttle = speedLimit * base.getLeftY();
			double turn = turnLimit *base.getRightX();
			
			//Driving Logic	
			drive.setOpenLoop(arcadeDriveHelper.arcadeDrive(throttle, -turn));
			boolean wantLowGear = shiftTrigger.get();
			drive.setHighGear(!wantLowGear);
			
			//Intake logic
			boolean openJaw = co.getLeftTrigger();
			boolean shoot = co.getRawButton(6);
			boolean shootFar = co.getRightTrigger();
			//boolean setArmLow = co.getRawButton(10);
			
			
			IntakeState wanted = new IntakeState();
			
			if(wantIntake) {
				wanted.setPower(0.75);
			}
			if(openJaw) {
				wanted.jawState = IntakeState.JawState.OPEN;
			} else if(intake.getCurrentState().detectedCube()){
				wanted.jawState = IntakeState.JawState.CLOSED; 
				wanted.setPower(0.0);
				//superstructure.setDesiredAngle(45.0);
			}
			
			if(holdIntake == false && intake.getCurrentState().detectedCube()) {
				holdIntake = true;
				superstructure.setDesiredAngle(40.0);
			} else if(holdIntake == true && !intake.getCurrentState().detectedCube()) {
				holdIntake = false;
			}
			
			/*else if(intake.getCurrentState().detectedCube() && !co.getRawButton(1)) {
				wanted.jawState = IntakeState.JawState.CLOSED; 
				wanted.setPower(0.0);
				superstructure.setDesiredAngle(45.0);
				
			}*/ 
			
			//Intake takes precedence over shooting far which takes precedence over shooting, so if you button mash
			//That's what to expect.
			if(shootFar) {
				wanted.setPower(-1.0);
				if(wanted.jawState == IntakeState.JawState.CLOSED) {
					wanted.jawState = IntakeState.JawState.NEUTRAL;
				}
				wantIntake = false;
			} else if(shoot) {
				wanted.setPower(-0.7);
				if(wanted.jawState == IntakeState.JawState.CLOSED) {
					wanted.jawState = IntakeState.JawState.NEUTRAL;
				}
				wantIntake = false;
			}
			
			//intake.setWantedState(wanted);
			
			//Climb
			if(co.getRawButton(8)) {
				double manualElevator = 0.5 * co.getLeftY();
				superstructure.setOpenLoopPower(manualElevator);
				//superstructure.setDesiredAngle(0.0);
				//wanted.jawState = IntakeState.JawState.OPEN;
				//wanted.setPower(0.0);
				//This was for manualArm control, but has since been removed.
				//double manualArm = -0.5 * co.getRightY();
				//arm.setOpenLoop(manualArm);
				
				//Climb mode
				if(co.getRawButton(5)) {
					superstructure.setDesiredAngle(0.0);
					//wanted.jawState = IntakeState.JawState.OPEN;
				}
			} else {
				if(co.getRawButton(1)) {
					//101.0 since due to slack, 90.0 goes to 80.0 degrees
					superstructure.setDesiredAngle(101.0);
					superstructure.setDesiredHeight(0.0);
					if(intake.getCurrentState().detectedCube()) {
						wanted.setPower(0.0);
					} else {
						wanted.setPower(0.75);
						wantIntake = true;
					}
					/*if(!holdIntake) {
						superstructure.setDesiredAngle(101.0);
					}*/
				} else if(co.getRawButton(2)) {
					superstructure.setDesiredAngle(101.0);
					superstructure.setDesiredHeight(30.0);
					wantIntake = false;
				} else if(co.getRawButton(4)) {
					superstructure.setDesiredAngle(38.0);
					superstructure.setDesiredHeight(63.5);
					wantIntake = false;
				} else if(co.getRawButton(3)) {
					superstructure.setDesiredAngle(0.0);
					wantIntake = false;
				}
			}
			
			if(intake.getCurrentState().detectedCube()) {
				leds.setState(LEDState.HAS_CUBE);
			} else if(wanted.leftMotor != 0.0) {
				leds.setState(LEDState.INTAKING);
			} else {
				leds.setState(LEDState.NO_CUBE);
			}
			
			intake.setWantedState(wanted);
			 
			allPeriodic();
		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}
	
	@Override
	public void disabledInit() {
		try {
			CrashTracker.logDisabledInit();
			
			if(autoModeExecuter != null) {
				autoModeExecuter.stop();
			}
			
			autoModeExecuter = null;
			
			internalLooper.stop();
			
			subsystemManager.stop();
			
			drive.setOpenLoop(DriveSignal.NEUTRAL);
			
		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}
	
	@Override
	public void disabledPeriodic() {
		if(base.getRawButton(1) || co.getRawButton(1)) {
			autoMode = AutoMode.SWITCH;
		} else if(base.getRawButton(2) || co.getRawButton(2)) {
			autoMode = AutoMode.RIGHT_START;
		} else if(base.getRawButton(3) || co.getRawButton(3)) {
			autoMode = AutoMode.LEFT_START;
		} else if(base.getRawButton(4) || co.getRawButton(4)) {
			autoMode = AutoMode.CROSS_AUTO_LINE;
		} else if(base.getRawButton(5) || co.getRawButton(5)) {
			autoMode = AutoMode.TEST;
		} else if (base.getRawButton(8) || co.getRawButton(8)) {
			autoMode = AutoMode.DO_NOTHING;
		}
		
		System.out.println(autoMode);
		
		allPeriodic();
	}
	
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	public void allPeriodic() {
		ConnectionMonitor.getInstance().setLastPacketTime(Timer.getFPGATimestamp());
		
		subsystemManager.outputToSmartDashboard();
		subsystemManager.writeToLog();
		internalLooper.outputToSmartDashboard();
	}
}
