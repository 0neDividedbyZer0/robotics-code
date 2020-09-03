package org.usfirst.frc.team1671.subsystems;

import org.usfirst.frc.team1671.loops.Loop;
import org.usfirst.frc.team1671.loops.Looper;
import org.usfirst.frc.team1671.robot.LatchedBoolean;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * @author Maverick Zhang
 * A class to that checks your connection to the robot. Requires LEDs for effectiveness.
 *
 */
public class ConnectionMonitor extends Subsystem {
	
	public static double connectionTimeout = 1.0; //seconds
	
	private static ConnectionMonitor instance = null;
	
	public static ConnectionMonitor getInstance() {
		if(instance == null) {
			instance = new ConnectionMonitor();
		}
		return instance;
	}
	
	private double lastPacketTime;
	private double dt;
	private LatchedBoolean justReconnected;
	private LatchedBoolean justDisconnected;
	
	private ConnectionMonitor() {
		lastPacketTime = 0.0;
		dt = 0.0;
		justReconnected = new LatchedBoolean();
		justDisconnected = new LatchedBoolean();
	}
	
	@Override
	public void registerEnabledLoops(Looper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				synchronized(ConnectionMonitor.this) {
					lastPacketTime = timestamp;
				}
			}
			
			@Override
			public void onLoop(double timestamp) {
				synchronized(ConnectionMonitor.this) {
					boolean has_connection = true;
					if(timestamp - lastPacketTime > connectionTimeout) {
						has_connection = false;
					}
					
					if(justReconnected.update(has_connection)) {
						//We need LEDs
					}
					
					if(justDisconnected.update(!has_connection)) {
						//We still need LEDs
					}
				}
			}
			
			@Override
			public void onStop(double timestamp) {
				
			}
		});
	}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("last packet time", dt);
	}

	@Override
	public void stop() {
	}

	@Override
	public void zeroSensors() {
	}
	
	public synchronized void setLastPacketTime(double timestamp) {
		dt = timestamp - lastPacketTime;
        lastPacketTime = timestamp;
    }
	
}
