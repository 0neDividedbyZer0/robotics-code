/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1671.robot;

import java.util.Arrays;

import org.usfirst.frc.team1671.auto.AutoModeExecuter;
import org.usfirst.frc.team1671.loops.CrashTracker;
import org.usfirst.frc.team1671.loops.Looper;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	//First you need every subsystem to be created
	//ex: private Drive drive = Drive.getInstance();
	private AutoModeExecuter autoModeExecuter = null;
	
	//A Subsystem Manager for Convenience
	//You should create this for major subsystems, or else they won't function.
	/*ex: private final SubsystemManager subsystemManager = new SubsystemManager(
			Arrays.asList(Drive.getInstance(), ConnectionMonitor.getInstance()));*/
	
	Looper internalLooper = new Looper();
	
	
	public Robot() {
		CrashTracker.logRobotConstruction();
	}
	
	public void zeroSensors() {
	}
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		try {
			
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
			
			System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());
			
			if (autoModeExecuter != null) {
                autoModeExecuter.stop();
            }
			
			zeroSensors();
			
			autoModeExecuter = null;
			
			internalLooper.start();
			autoModeExecuter = new AutoModeExecuter();
			autoModeExecuter.setAutoMode(AutoModeSelector.getSelectedAutoMode());
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
			
			internalLooper.start();

			zeroSensors();
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
			
		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}
	
	@Override
	public void disabledPeriodic() {
		zeroSensors();
		allPeriodic();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	public void allPeriodic() {
		internalLooper.outputToSmartDashboard();
	}
}
