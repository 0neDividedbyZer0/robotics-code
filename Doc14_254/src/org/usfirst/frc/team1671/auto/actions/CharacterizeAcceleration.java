package org.usfirst.frc.team1671.auto.actions;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team1671.robot.DriveSignal;
import org.usfirst.frc.team1671.subsystems.Drive;

import edu.wpi.first.wpilibj.Timer;
import trajectory_lib.Coord;
import trajectory_lib.ReflectingCSVAppender;
import trajectory_lib.ReflectingCSVWriter;
import util.LinearRegression;
import util.Util;

public class CharacterizeAcceleration implements Action {
	private Drive drive = Drive.getInstance();
	private List<Coord> accelerationData;
	private double prevVel;
	private double prevTime;
	private double kv, ks;
	private ReflectingCSVWriter<Coord> writer;
	
	public CharacterizeAcceleration(double kv, double ks) {
		accelerationData = new ArrayList<Coord>();
		this.kv = kv;
		this.ks = ks;
		writer = new ReflectingCSVWriter<Coord>("/home/lvuser/paths/accelData.csv", Coord.class);
	}

	@Override
	public boolean isFinished() {
		if(drive.getLeftEnc() >= 100.0 || drive.getRightEnc() >= 100.0) {
			return true;
		}
		return false;
	}

	@Override
	public void update() {
		double currVel = drive.getVelocity();
		double currTime = Timer.getFPGATimestamp();
		
		double acc = (currVel - prevVel) / (currTime - prevTime);
		
		/*if(Util.epsilonEquals(acc, 0.0, Util.kEpsilon)) {
			prevVel = currVel;
			prevTime = currTime;
			return;
		}*/
		
		Coord coord = new Coord();
		coord.x = acc;
		coord.y = 12.0 - ks - kv * currVel;
		
		accelerationData.add(coord);
		
		writer.add(coord);
		
		//System.out.println("V: " + coord.y + " Acc: " + coord.x);
		
		prevVel = currVel;
		prevTime = currTime;
	}

	@Override
	public void done() {
		drive.setOpenLoop(DriveSignal.BRAKE);
		writer.flush();
		
		LinearRegression regression = new LinearRegression(accelerationData);
		System.out.println("accel a: " + regression.a + " b: " + regression.b);		
		
		
	}

	@Override
	public void start() {
		prevTime = Timer.getFPGATimestamp();
		prevVel = drive.getVelocity();
		drive.setOpenLoop(new DriveSignal(1.0, 1.0, true));
	}

}
