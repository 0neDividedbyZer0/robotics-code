package org.usfirst.frc.team1671.auto.actions;

import org.usfirst.frc.team1671.states.SuperstructureState;
import org.usfirst.frc.team1671.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Timer;
import util.Util;

public class SuperstructurePosition implements Action {
	private static final Superstructure superstructure = Superstructure.getInstance();
	private static final double heightEpsilon = 2.0;
	private static final double angleEpsilon = 5.0;
	private static final double timeout = 4.0;
	
	private final double height, angle;
	private final boolean waitDone;
	private double startTime;
	
	public SuperstructurePosition(double height, double angle, boolean waitDone) {
		this.height = height;
		this.angle = angle;
		//WaitDone will block the next action from proceeding until this action finishes
		this.waitDone = waitDone;
	}
	
	@Override
	public void start() {
		superstructure.setDesiredHeight(height);
		superstructure.setDesiredAngle(angle);
		startTime = Timer.getFPGATimestamp();
	}
	
	@Override
	public void update() {
	}
	
	@Override
	public boolean isFinished() {
		if(Timer.getFPGATimestamp() - startTime > timeout) {
			System.out.println("Superstructure Timed Out");
			return true;
		}
		if(waitDone) {
			SuperstructureState state = superstructure.getObservedState();
			return Util.epsilonEquals(state.height, height, heightEpsilon) 
					&& Util.epsilonEquals(state.angle, angle, angleEpsilon);
		} else {
			return true;
		}
	}

	@Override
	public void done() {
	}

	

}
