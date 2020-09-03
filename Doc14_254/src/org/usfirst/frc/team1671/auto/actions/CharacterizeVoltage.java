package org.usfirst.frc.team1671.auto.actions;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team1671.robot.DriveSignal;
import org.usfirst.frc.team1671.subsystems.Drive;

import edu.wpi.first.wpilibj.Timer;
import trajectory_lib.Coord;
import util.LinearRegression;
import util.Util;

public class CharacterizeVoltage implements Action {
	private Drive drive = Drive.getInstance();
	private double startTime;
	private List<Coord> left, right, center;
	private LinearRegression regression;
	private static final double VperS = 0.25;
	private static final double percentPerS = VperS / 12.0;
	
	public CharacterizeVoltage() {
		left = new ArrayList<Coord>();
		right = new ArrayList<Coord>();
		center = new ArrayList<Coord>();
	}

	@Override
	public boolean isFinished() {
		if(drive.getLeftEnc() >= 200.0 || drive.getRightEnc() >= 200.0) {
			return true;
		}
		return false;
	}

	@Override
	public void update() {
		if(drive.getLeftEnc() < 200.0 || drive.getRightEnc() < 200.0) {
			double currentTime = Timer.getFPGATimestamp();
			double currentSecond = Math.floor(10.0 * (currentTime - startTime)) / 10.0 + 0.100;
			double currentVolts = currentSecond * VperS;
			double currentSignal = currentSecond * percentPerS;
			
			DriveSignal signal = new DriveSignal(currentSignal, currentSignal, true);
			drive.setOpenLoop(signal);
			
			double leftSpeed = drive.getLeftEncRate();
			double rightSpeed = drive.getRightEncRate();
			double centerSpeed = (leftSpeed + rightSpeed) / 2.0;
			
			Coord leftCoord = new Coord();
			Coord rightCoord = new Coord();
			Coord centerCoord = new Coord();
			
			leftCoord.x = currentVolts;
			rightCoord.x = currentVolts;
			centerCoord.x = currentVolts;
			
			leftCoord.y = leftSpeed;
			rightCoord.y = rightSpeed;
			centerCoord.y = centerSpeed;
			
			left.add(leftCoord);
			right.add(rightCoord);
			center.add(centerCoord);
		} else {
			drive.setOpenLoop(DriveSignal.BRAKE);
		
		
		
		
			List<Coord> prunedList = new ArrayList<Coord>();
			for(Coord coord : left) {
				if(!Util.epsilonEquals(coord.y, 0.0, Util.kEpsilon)) {
					prunedList.add(coord);
				}
			}
			regression = new LinearRegression(prunedList);
			System.out.println("left: " + regression.a + " right: " + regression.b);
			prunedList = new ArrayList<Coord>();
			for(Coord coord : right) {
				if(!Util.epsilonEquals(coord.y, 0.0, Util.kEpsilon)) {
					prunedList.add(coord);
				}
			}
			regression = new LinearRegression(prunedList);
			System.out.println("right: " + regression.a + " right: " + regression.b);
			prunedList = new ArrayList<Coord>();
			for(Coord coord : center) {
				if(!Util.epsilonEquals(coord.y, 0.0, Util.kEpsilon)) {
					prunedList.add(coord);
				}
			}
			regression = new LinearRegression(prunedList);
			System.out.println("center: " + regression.a + " right: " + regression.b);
		
		}
		
		
		
	}

	@Override
	public void done() {
		drive.setOpenLoop(DriveSignal.BRAKE);
		
		List<Coord> prunedList = new ArrayList<Coord>();
		for(Coord coord : left) {
			if(!Util.epsilonEquals(coord.y, 0.0, Util.kEpsilon)) {
				prunedList.add(coord);
			}
		}
		regression = new LinearRegression(prunedList);
		System.out.println("left: " + regression.a + " right: " + regression.b);
		prunedList = new ArrayList<Coord>();
		for(Coord coord : right) {
			if(!Util.epsilonEquals(coord.y, 0.0, Util.kEpsilon)) {
				prunedList.add(coord);
			}
		}
		regression = new LinearRegression(prunedList);
		System.out.println("right: " + regression.a + " right: " + regression.b);
		prunedList = new ArrayList<Coord>();
		for(Coord coord : center) {
			if(!Util.epsilonEquals(coord.y, 0.0, Util.kEpsilon)) {
				prunedList.add(coord);
			}
		}
		regression = new LinearRegression(prunedList);
		System.out.println("center: " + regression.a + " right: " + regression.b);
	}

	@Override
	public void start() {
		startTime = Timer.getFPGATimestamp();
		System.out.println("began");
	}

}
