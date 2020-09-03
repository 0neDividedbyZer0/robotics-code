package org.usfirst.frc.team1671.subsystems;

import org.usfirst.frc.team1671.loops.CrashTracker;
import org.usfirst.frc.team1671.loops.Loop;
import org.usfirst.frc.team1671.loops.Looper;
import org.usfirst.frc.team1671.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDs extends Subsystem {
	private static LEDs instance;
	
	public static LEDs getInstance() {
		if(instance == null) {
			instance = new LEDs();
		}
		
		return instance;
	}
	private final DigitalOutput state1, state2, state3;
	
	//private final SerialPort serial;
	
	//private boolean connected;
	
	private LEDState state;
	
	public enum LEDState {
		INIT,
		HAS_CUBE,
		INTAKING,
		NO_CUBE
	}
	
	private LEDs() {
		try {
			//serial = new SerialPort(RobotMap.BAUD_RATE, SerialPort.Port.kUSB1);
			//connected = true;
			
			state1 = new DigitalOutput(RobotMap.LEDsDIO1);
			state2 = new DigitalOutput(RobotMap.LEDsDIO2);
			state3 = new DigitalOutput(RobotMap.LEDsDIO3);
		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			//connected = false;
			throw t;
		}
	}
	
	/*private synchronized String read() {
		return serial.readString();
	}
	
	private synchronized void write(String data) {
		serial.writeString(data);
	}*/
	
	public synchronized void setState(LEDState wantedState) {
		if(state != wantedState) {
			state = wantedState;
		}
	}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putString("LEDState", "" + state);
	}

	@Override
	public void stop() {
		
	}

	@Override
	public void zeroSensors() {
	}

	@Override
	public void registerEnabledLoops(Looper enabledLooper) {
		enabledLooper.register(new Loop() {

			@Override
			public void onStart(double timestamp) {
			}

			@Override
			public void onLoop(double timestamp) {
				
				synchronized(LEDs.this) {
					switch(state) {
					case INIT:
						state1.set(false);
						state2.set(false);
						state3.set(false);
						break;
					case NO_CUBE:
						state1.set(true);
						state2.set(false);
						state3.set(false);
						break;
					case INTAKING:
						state1.set(false);
						state2.set(true);
						state3.set(false);
						break;
					case HAS_CUBE:
						state1.set(true);
						state2.set(true);
						state3.set(false);
						break;
					default:
						System.out.println("LEDs in unexpected state: " + state);
					}
					
				}
			}

			@Override
			public void onStop(double timestamp) {
			}
			
		});
	}

}
