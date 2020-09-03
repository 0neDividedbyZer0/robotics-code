package org.usfirst.frc.team1671.robot;

//speed, acceleration to applied voltage
public class VAAVConverter {
	private double ks, kv, ka;
	
	public VAAVConverter(double ks, double kv, double ka) {
		this.ks = ks;
		this.kv = kv;
		this.ka = ka;
	}
	
	public double convert(double vel, double acc) {
		return ks + kv * vel + ka * acc;
	}
	
	public double ks() {
		return ks;
	}
	
	public double kv() {
		return kv;
	}
	
	public double ka() {
		return ka;
	}
}
