package org.usfirst.frc.team1671.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Trigger;

public class AxisGreater extends Trigger {
	
	private Joystick m_stick;
	private int m_axis;
	private double m_value;
	
	public AxisGreater(Joystick stick, int axis, double value) {
		m_stick = stick;
		m_axis = axis;
		m_value = value;
	}

	@Override
	public boolean get() {
		return m_stick.getRawAxis(m_axis) > m_value;
	}
}