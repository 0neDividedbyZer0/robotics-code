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

class linear_actuator(state_space_controller):
  def __init__(self, dt, name, num_stages, num_motors, stall_current, free_current, stall_torque,
               free_speed, moment_of_inertia, gear_ratio, pitch, load_torque, C = numpy.matrix([[1 , 0]])):
    state_space_controller.__init__(self, dt, name)
    """Instantiates the arm template. Units should be SI. Free speed 
    should be radians per second, gear ratio should be greater than one, 
    stall current, free current, stall torque just refer to your motor specs for one
    of that type which should be found on Vex Motors. The dt refers to the robot's
    loop speed which as of 2018, is 5 ms (so dt = 0.005), and name refers to what
    you want to call the subsystem."""
    
    #Input values
    self.N = num_stages
    self.n = num_motors
    self.I_stall = stall_current * num_motors
    self.I_free = free_current * num_motors
    self.tau_stall = stall_torque * num_motors
    self.omega_free = free_speed
    self.J = moment_of_inertia
    self.G = gear_ratio
    self.p = pitch * 2.0 * math.pi  #convention is rev / in
    self.tau_l = load_torque
    
    
    #Derived Values
    #efficiency
    self.mu = 1.0#0.95 ** self.N 
    #resistance
    self.R = 12.0 / self.I_stall
    self.k_t = self.I_stall / self.tau_stall
    self.k_v = (12.0 - self.I_free * self.R) / self.omega_free
    #coefficient of velocity
    self.k_omega = -self.k_v * self.G * self.G/ (self.k_t * self.R * self.J)
    #coefficient of input
    self.k_u = self.mu * self.G / (self.k_t * self.R * self.J * self.p)
    
    self.v_load_factor = self.k_t * self.R * self.tau_l / self.G
    
    #State is [[angle],[angular_velocity]]
    self.A_c = numpy.matrix([[0 , 1],
                             [0 , self.k_omega]])
    self.B_c = numpy.matrix([[0],
                             [self.k_u]])
    
    self.C = C
    self.D = numpy.matrix([[0]])
    
    #converts continuous to discrete matrices
    self.A_d, self.B_d = controls.c2d(self.A_c,self.B_c,self.dt)
    
    self.minU = numpy.matrix([[-12.0]])
    self.maxU = numpy.matrix([[12.0]])
    
    self.initialize_state() 
    
  def run_test(self, initial_X, goal, show_graph=True, use_observer=True, inches = True, iterations=4000):
    """This method will run a default test based on LQR or pole-placement. You need
    to set K to something to run this or you will get an error. For fabrication or
    CAD, you should NOT be using this method."""
    self.X = initial_X
    self.X_hat = self.X
    if use_observer:
      self.X_hat = initial_X + 0.03
    t = []
    theta = []
    omega = []
    theta_hat = []
    omega_hat = []
    u = []
      
  #  sep_plot_gain - 100.0
    
    if inches:
      unchanged_goal = goal * 0.0254
    else:
      unchanged_goal = goal
    
    for i in range(iterations):
      if not use_observer:
        self.X_hat = self.X
      if use_observer:
        if inches:
          theta_hat.append(self.X_hat[0,0] * 39.3701)
          omega_hat.append(self.X_hat[1,0] * 39.3701)
        else:
          theta_hat.append(self.X_hat[0,0])
          omega_hat.append(self.X_hat[1,0])
      U = self.K * (unchanged_goal - self.X_hat)
      U = numpy.clip(U, self.minU, self.maxU)
      if inches:
        theta.append(self.X[0,0] * 39.3701)
        omega.append(self.X[1,0] * 39.3701)
      else:
        theta.append(self.X[0,0])
        omega.append(self.X[1,0])
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
      pyplot.plot(t, theta, label='length')
      pyplot.plot(t, omega, label='velocity')
      print("Final State: ")
      if inches:
        print(self.X * 39.3701)
      else:
        print(self.X)
      print("\n")
      if use_observer:
        pyplot.plot(t, theta_hat, label='x_hat length')
        pyplot.plot(t, omega_hat, label='x_hat velocity')
        print("Final Estimated State: ")
        if inches:
          print(self.X_hat * 39.3701)
        else:
          print(self.X_hat)
      print("\n")
      pyplot.legend()
    
      pyplot.subplot(2, 1, 2)
      pyplot.plot(t, u, label='voltage')
      pyplot.legend()
      pyplot.show()
    
  def run_pid_test(self, constants, initial_X, goal, show_graph=True, inches= True, iterations=4000):
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
    theta = []
    omega = []
    error = []
    u = []
    
    for i in range(iterations):
      if inches:
        U = [controller.Update(self.X[0,0]* 39.3701)]
      else:
        U = [controller.Update(self.X[0,0])]
      U = numpy.clip(U, self.minU, self.maxU)
      if inches:
        theta.append(self.X[0,0]* 39.3701)
        omega.append(self.X[1,0]* 39.3701)
        error.append((goal[0,0]-self.X[0,0])* 39.3701)
      else:
        theta.append(self.X[0,0])
        omega.append(self.X[1,0])
        error.append(goal[0,0]-self.X[0,0])
      self.update(U)
      t.append(i * self.dt)
      u.append(U[0,0])
      
    if show_graph:
      fig_size = [12, 9]
      pyplot.rcParams["figure.figsize"] = fig_size
      pyplot.subplot(2, 1, 1)
      pyplot.plot(t, theta, label='length')
      pyplot.plot(t, omega, label='velocity')
      print("Final State: ")
      if inches:
        print(self.X * 39.3701)
      else:
        print(self.X)
      print("\n")
      #pyplot.plot(t, error, label='error')
      pyplot.legend()
    
      pyplot.subplot(2, 1, 2)
      pyplot.plot(t, u, label='voltage')
      pyplot.legend()
      pyplot.show()
      

  def run_custom_test(self, initial_X, goal, f, show_graph=True, use_observer=True, inches = True, iterations=4000):
    """This method runs a custom test on the system. f should be a user defined
    function. deg refers to whether or not
    you want to convert to degrees on the graph, it will be in radians if set to
    false. Iterations means how many loops to run. For a dt = 0.005, 4000 iterations
    is 20 seconds. A different dt will create a different length test for a given
    number of iterations. Smaller dt run more precisely, but consume more resources.
    Larger dt run with less resources, but are less accurate. use_observer should be
    False for any CAD and Fab users."""
    self.X = initial_X
    self.X_hat = self.X
    if use_observer:
      self.X_hat = initial_X + 0.03
    t = []
    theta = []
    omega = []
    theta_hat = []
    omega_hat = []
    torque = []
    u = []
      
  #  sep_plot_gain - 100.0
    
    if inches:
      unchanged_goal = goal * 0.0254
    else:
      unchanged_goal = goal
    
    for i in range(iterations):
      if not use_observer:
        self.X_hat = self.X
      if use_observer:
        if inches:
          theta_hat.append(self.X_hat[0,0] * 39.3701)
          omega_hat.append(self.X_hat[1,0] * 39.3701)
        else:
          theta_hat.append(self.X_hat[0,0])
          omega_hat.append(self.X_hat[1,0])
      U = f(unchanged_goal - self.X_hat)
      U = numpy.clip(U, self.minU, self.maxU)
      u.append(U[0,0])
      torque.append((U[0,0] - self.k_v * self.G * self.p * self.X[1,0])/(self.k_t* self.R))
      U -= self.v_load_factor
      
      #torque = self.VtoT(self.X, U[0,0])
      if inches:
        theta.append(self.X[0,0] * 39.3701)
        omega.append(self.X[1,0] * 39.3701)
      else:
        theta.append(self.X[0,0])
        omega.append(self.X[1,0])
      if use_observer:
        self.predict_observer(U)
      #self.X = self.ddynamics(self.X, torque)
      #self.Y = self.C * self.X + self.D * U
      self.update(U)
      if use_observer:
        self.correct_observer(U)
      t.append(i * self.dt)
      
      
    if show_graph:
      fig_size = [12, 9]
      pyplot.rcParams["figure.figsize"] = fig_size
      pyplot.subplot(2, 1, 1)
      pyplot.plot(t, theta, label='length')
      pyplot.plot(t, omega, label='velocity')
      pyplot.plot(t, torque, label='torque')
      print("Final State: ")
      if inches:
        print(self.X *39.3701)
      else:
        print(self.X)
      print("\n")
      if use_observer:
        pyplot.plot(t, theta_hat, label='x_hat length')
        pyplot.plot(t, omega_hat, label='x_hat velocity')
        print("Final Estimated State: ")
        if inches:
          print(self.X_hat * 39.3701)
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
    
  def get_stats(self, inches = True):
    """This method will get you the theoretical maximums of the system."""
    self.max_vel = -self.k_u * (12.0 - self.v_load_factor) / self.k_omega
    self.max_acc = self.k_u * (12.0 - self.v_load_factor)
    torque_steady = self.tau_l / self.G
    if inches:
      self.max_vel *= 39.3701
      self.max_acc *= 39.3701
    print("Maximum Velocity: " + str(self.max_vel) + "\n")
    print("Maximum Acceleration: " + str(self.max_acc) + "\n")
    print("Load Factor: " + str(self.v_load_factor) + "\n") 
    print("Steady-State Torque: " + str(torque_steady))

    

