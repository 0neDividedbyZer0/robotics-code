# -*- coding: utf-8 -*-
"""
Created on Thu Apr 26 22:04:37 2018

@author: Maverick1
"""

class PIDController():
  def __init__(self, kp, ki, kd, dt, setpoint):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.dt = dt
    self.integral = 0
    self.error = 0
    self.prev_error = 0
    self.setpoint = setpoint
    
  def reset(self):
    self.integral = 0
    self.error = 0
    self.prev_error = 0
    
  def setSetpoint(self, setpoint):
    self.setpoint = setpoint

  def Update(self, curr):
    self.prev_error = self.error
    self.error = self.setpoint - curr
    self.integral += self.error * self.dt
    return self.kp * self.error + self.ki * self.integral + self.kd * (self.error - self.prev_error) / self.dt