# -*- coding: utf-8 -*-
"""
Created on Thu May  3 17:08:45 2018

@author: Maverick Zhang
"""
from state_space_controller import StateSpaceController
import math
import numpy
from matplotlib import pyplot

class Elevator(StateSpaceController):
  def __init__(self, dt):
    StateSpaceController.__init__(self, dt)
    #free speed in radians per second
    self.free_speed = 18730.0 * 2.0 * math.pi / 60.0
    #free current in amps, 4 since there are 4 motors
    self.free_current = 4.0 * 0.7
    #Newton meters, 2 motors on each side
    self.stall_torque = 2.0 * 0.71 
    #amps, there are 4 motors
    self.stall_current = 4.0 * 134.0
    #mass in kilograms
    self.mass = 15.0 * 0.453592
    
    #Ohms
    self.R = 12.0 / self.stall_current
    self.Kv = (12.0 - self.free_current * self.R) / self.free_speed
    self.Kt = self.stall_torque / self.stall_current
    self.G = 40.62
    #pulley radius in inches
    self.r = 1.751 / 2.0
    #moment of inertia
    self.J = 0.5 * self.mass * self.r * self.r
    
    self.a = (-self.G * self.G * self.Kt * self.Kv) / (self.R * self.r * self.r * 0.5 * self.mass)
    self.b = (self.Kt * self.G) / (2.0 * 0.5 * self.mass * self.r * self.R)
    #state is [average position, average velocity
    #            position difference / 2.0, velocity difference / 2.0]
    #input is [left voltage, right voltage]
    #output is [left position, right position]
    self.A_c = numpy.asmatrix([[0 , 1, 0, 0],
                               [0, self.a, 0, 0],
                               [0, 0, 0, 1],
                               [0, 0, 0, self.a]])
    
    self.B_c = numpy.asmatrix([[0, 0],
                               [self.b / 2.0, self.b / 2.0],
                               [0, 0],
                               [self.b / 2.0, -self.b / 2.0]])
    
    self.C = numpy.asmatrix([[1, 0, 1, 0],
                             [1, 0, -1, 0]])
    
    self.D = numpy.asmatrix([[0, 0],
                             [0, 0]])
    
    self.ContinuousToDiscrete()
    
    self.PlaceControllerPoles((-0.5, -0.6, 0.97 , 0.97))
    
    self.PlaceObserverPoles((0.2, -0.2, 0.1, -0.1))
    
    self.minU = numpy.asmatrix([[-12.0, -12.0]])
    self.maxU = numpy.asmatrix([[12.0, 12.0]])
    
    self.InitializeState()
    
  def run_test(self, initial_X, goal, show_graph=True, use_observer=True, iterations=4000):
    self.X = initial_X
    self.X_hat = self.X
    if use_observer:
      self.X_hat = initial_X + 0.03
    t = []
    pos = []
    vel = []
    pos_diff = []
    vel_diff = []
    pos_hat = []
    vel_hat = []
    pos_diff_hat = []
    vel_diff_hat = []
    uL = []
    uR = []
      
    #sep_plot_gain = 100.0
    
    unchanged_goal = goal
    
    for i in range(iterations):
      if not use_observer:
        self.X_hat = self.X
      if use_observer:
        pos_hat.append(self.X_hat[0,0])
        vel_hat.append(self.X_hat[1,0])
        pos_diff_hat.append(self.X_hat[2,0])
        vel_diff_hat.append(self.X_hat[3,0])
      U = self.K * (unchanged_goal - self.X_hat)
      U = numpy.clip(U, self.minU, self.maxU)
      pos.append(self.X[0,0])
      vel.append(self.X[1,0])
      pos_diff.append(self.X[2,0])
      vel_diff.append(self.X[3,0])
      if use_observer:
        self.PredictObserver(U)
      self.Update(U)
      if use_observer:
        self.CorrectObserver(U)
      t.append(i * self.dt)
      uL.append(U[0,0])
      uR.append(U[1,0])
      
    if show_graph:
      fig_size = [12, 9]
      pyplot.rcParams["figure.figsize"] = fig_size
      pyplot.subplot(2, 1, 1)
      pyplot.plot(t, pos, label='pos')
      pyplot.plot(t, vel, label='vel')
      pyplot.plot(t, pos_diff, label='pos_diff')
      pyplot.plot(t, vel_diff, label='vel_diff')
      if use_observer:
        pyplot.plot(t, pos_hat, label='x_hat pos')
        pyplot.plot(t, vel_hat, label='x_hat vel')
        pyplot.plot(t, pos_diff_hat, label='x_hat pos_diff')
        pyplot.plot(t, vel_diff, label='x_hat vel_diff')
      pyplot.legend()
    
      pyplot.subplot(2, 1, 2)
      pyplot.plot(t, uL, label='voltage left')
      pyplot.plot(t, uR, label='voltage right')
      pyplot.legend()
      pyplot.show()
    
def main():
  elevator = Elevator(0.02)
  elevator.run_test(numpy.matrix([[0.0],[0.0],[0.5j],[-0.5j]]), numpy.matrix([[48.0],[0.0],[0.0],[0.0]]), True, True, 3000)
  #elevator.export("elevatorpy")
              
    
if __name__ == "__main__":
  main()  
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
