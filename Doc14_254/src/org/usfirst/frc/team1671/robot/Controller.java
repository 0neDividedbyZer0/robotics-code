package org.usfirst.frc.team1671.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class Controller extends Joystick{
	
	public Controller(int port){
		super(port);
	}
	
	
	public double getLeftX(){
		
		return getRawAxis(0);
	}
	
	public double getLeftY(){
		
		return getRawAxis(1);
	}
	
	public double getRightX(){
		
		return getRawAxis(4);
	}
	
	public double getRightY(){
		
		return -getRawAxis(5);
	}
	
	public boolean getLeftTrigger(){
		return getRawAxis(2) > 0.3;
	}
	
	public boolean getRightTrigger() {
		return getRawAxis(3) > 0.3;
	}
	
	public void setRumble(boolean on) {
		setRumble(GenericHID.RumbleType.kRightRumble, on ? 1.0 : 0.0);
	}
	
	public boolean getReadyClimbPOV() {
		return getPOV() == 0;
	}

}