# -*- coding: utf-8 -*-
"""
Created on Sat Nov  3 19:48:48 2018

@author: Maverick Zhang
"""

import numpy
from matplotlib import pyplot
from state_space_controller import state_space_controller
import controls
import pid_controller
import math

class shooter(state_space_controller):
  def __init__(self, dt, name, num_stages, num_motors, stall_current, free_current, stall_torque,
               free_speed, moment_of_inertia, gear_ratio, C = numpy.matrix([[1]])):
    """Instantiates the shooter template. Units should be SI. Free speed 
    should be radians per second, gear ratio should be greater than one, 
    stall current, free current, stall torque just refer to your motor specs for one
    of that type which should be found on Vex Motors. The dt refers to the robot's
    loop speed which as of 2018, is 5 ms (so dt = 0.005), and name refers to what
    you want to call the subsystem."""
    state_space_controller.__init__(self, dt, name)
    #Input values
    self.N = num_stages
    self.n = num_motors
    self.I_stall = stall_current * num_motors
    self.I_free = free_current * num_motors
    self.tau_stall = stall_torque * num_motors
    self.omega_free = free_speed
    self.J = moment_of_inertia
    self.G = gear_ratio
    
    #Derived Values
    #efficiency
    self.mu = 0.95 ** self.N 
    #resistance
    self.R = 12.0 / self.I_stall
    self.k_t = self.I_stall / self.tau_stall
    self.k_v = (12.0 - self.I_free * self.R) / self.omega_free
    #coefficient of velocity
    self.k_omega = -self.k_v * self.G * self.G/ (self.k_t * self.R * self.J)
    #coefficient of input
    self.k_u = self.mu * self.G / (self.k_t * self.R * self.J)
    
    #State is [[angular_velocity]]
    self.A_c = numpy.matrix([[self.k_omega]])
    self.B_c = numpy.matrix([[self.k_u]])
    
    self.C = C
    self.D = numpy.matrix([[0]])
    
    #converts continuous to discrete matrices
    self.A_d, self.B_d = controls.c2d(self.A_c,self.B_c,self.dt)
    
    self.minU = numpy.matrix([[-12.0]])
    self.maxU = numpy.matrix([[12.0]])
    
    self.initialize_state() 
    
  def run_test(self, initial_X, goal, show_graph=True, use_observer=True, rpm = True, iterations=4000):
    self.X = initial_X
    self.X_hat = self.X
    if use_observer:
      self.X_hat = initial_X + 0.03
    t = []
    omega = []
    omega_hat = []
    u = []
      
  #  sep_plot_gain - 100.0
    
    if rpm:
      unchanged_goal = goal * math.pi / 30.0
    else:
      unchanged_goal = goal
    
    for i in range(iterations):
      if not use_observer:
        self.X_hat = self.X
      if use_observer:
        if rpm:
          omega_hat.append(self.X_hat[0,0] * 30. / math.pi)
        else:
          omega_hat.append(self.X_hat[0,0])
      U = self.K * (unchanged_goal - self.X_hat)
      U = numpy.clip(U, self.minU, self.maxU)
      if rpm:
        omega.append(self.X[0,0] * 30. / math.pi)
      else:
        omega.append(self.X[0,0])
      if use_observer:
        self.predict_observer(U)
      self.update(U)
      if use_observer:
        self.correct_observer(U)
      t.append(i * self.dt)
      u.append(U[0,0])
      
    if show_graph:
      fig_size = [12, 9]
      pyplot.rcParams["figure.figsize"] = fig_size
      pyplot.subplot(2, 1, 1)
      pyplot.plot(t, omega, label='angular velocity')
      print("Final State: ")
      if rpm:
        print(self.X * 30.0 / math.pi)
      else:
        print(self.X)
      print("\n")
      if use_observer:
        pyplot.plot(t, omega_hat, label='x_hat angular velocity')
        print("Final Estimated State: ")
        if rpm:
          print(self.X_hat * 30.0 / math.pi)
        else:
          print(self.X_hat)
      print("\n")
      pyplot.legend()
    
      pyplot.subplot(2, 1, 2)
      pyplot.plot(t, u, label='voltage')
      pyplot.legend()
      pyplot.show()
    
  def run_pid_test(self, constants, initial_X, goal, show_graph=True, rpm= True, iterations=4000):
    """This method runs a PID controller test on the system. Constants should be
    an array with P, I, D constants in that order. deg refers to whether or not
    you want to convert to degrees on the graph, it will be in radians if set to
    false. Iterations means how many loops to run. For a dt = 0.005, 4000 iterations
    is 20 seconds. A different dt will create a different length test for a given
    number of iterations. Smaller dt run more precisely, but consume more resources.
    Larger dt run with less resources, but are less accurate."""
    self.X = initial_X
    controller = pid_controller.PIDController(constants[0], constants[1], constants[2], self.dt, goal[0,0])
    t = []
    omega = []
    error = []
    u = []
    
    for i in range(iterations):
      if rpm:
        U = [controller.Update(self.X[0,0]* 30.0 / math.pi)]
      else:
        U = [controller.Update(self.X[0,0])]
      U = numpy.clip(U, self.minU, self.maxU)
      if rpm:
        omega.append(self.X[0,0]* 30.0 / math.pi)
        error.append((goal[0,0]-self.X[0,0])* 30.0 / math.pi)
      else:
        omega.append(self.X[0,0])
        error.append(goal[0,0]-self.X[0,0])
      self.update(U)
      t.append(i * self.dt)
      u.append(U[0,0])
      
    if show_graph:
      fig_size = [12, 9]
      pyplot.rcParams["figure.figsize"] = fig_size
      pyplot.subplot(2, 1, 1)
      pyplot.plot(t, omega, label='angular velocity')
      print("Final State: ")
      if rpm:
        print(self.X * 30.0 / math.pi)
      else:
        print(self.X)
      print("\n")
      #pyplot.plot(t, error, label='error')
      pyplot.legend()
    
      pyplot.subplot(2, 1, 2)
      pyplot.plot(t, u, label='voltage')
      pyplot.legend()
      pyplot.show()
      

  def run_custom_test(self, initial_X, goal, f, show_graph=True, use_observer=True, rpm = True, iterations=4000):
    """This method runs a custom test on the system. f should be a user defined
    function. deg refers to whether or not you want to convert to degrees on the 
    graph, it will be in radians if set to false. Iterations means how many loops 
    to run. For a dt = 0.005, 4000 iterations is 20 seconds. A different dt will 
    create a different length test for a given
    number of iterations. Smaller dt run more precisely, but consume more resources.
    Larger dt run with less resources, but are less accurate. use_observer should
    be set to false for Fabrication and CAD users"""
    self.X = initial_X
    self.X_hat = self.X
    if use_observer:
      self.X_hat = initial_X + 0.03
    t = []
    omega = []
    omega_hat = []
    u = []
      
  #  sep_plot_gain - 100.0
    
    if rpm:
      unchanged_goal = goal * math.pi / 30.0
    else:
      unchanged_goal = goal
    
    for i in range(iterations):
      if not use_observer:
        self.X_hat = self.X
      if use_observer:
        if rpm:
          omega_hat.append(self.X_hat[0,0] * 30. / math.pi)
        else:
          omega_hat.append(self.X_hat[0,0])
      U = f(unchanged_goal - self.X_hat)
      U = numpy.clip(U, self.minU, self.maxU)
      #torque = self.VtoT(self.X, U[0,0])
      if rpm:
        omega.append(self.X[0,0] * 30. / math.pi)
      else:
        omega.append(self.X[0,0])
      if use_observer:
        self.predict_observer(U)
      #self.X = self.ddynamics(self.X, torque)
      #self.Y = self.C * self.X + self.D * U
      self.update(U)
      if use_observer:
        self.correct_observer(U)
      t.append(i * self.dt)
      u.append(U[0,0])
      
    if show_graph:
      fig_size = [12, 9]
      pyplot.rcParams["figure.figsize"] = fig_size
      pyplot.subplot(2, 1, 1)
      pyplot.plot(t, omega, label='angular velocity')
      print("Final State: ")
      if rpm:
        print(self.X * 30.0 / math.pi)
      else:
        print(self.X)
      print("\n")
      if use_observer:
        pyplot.plot(t, omega_hat, label='x_hat angular velocity')
        print("Final Estimated State: ")
        if rpm:
          print(self.X_hat * 30.0 / math.pi)
        else:
          print(self.X_hat)
        print("\n")
      pyplot.legend()
    
      pyplot.subplot(2, 1, 2)
      pyplot.plot(t, u, label='voltage')
      pyplot.legend()
      pyplot.show()
      
  def set_K(self, K):
    self.K = numpy.asmatrix(K)

  def set_L(self, L):
    self.L = numpy.asmatrix(L)
    
  def get_stats(self, rpm = True):
    """This method will get you the theoretical maximums of the system."""
    self.max_vel = -self.k_u * 12.0 / self.k_omega
    self.max_acc = self.k_u * 12.0
    if rpm:
      self.max_vel *= 30.0 / math.pi
      self.max_acc *= 30.0 / math.pi
    print("Maximum Angular Velocity: " + str(self.max_vel) + "\n")
    print("Maximum Angular Acceleration: " + str(self.max_acc) + "\n")

    

