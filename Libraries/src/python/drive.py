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

class drive(state_space_controller):
  def __init__(self, dt, name, num_stages_per_side, num_motors_per_side, stall_current, free_current, stall_torque,
               free_speed, gear_ratio_per_side, moment_of_inertia, wheel_radius, drivebase_radius, mass,
               C = numpy.matrix([[1 , 0, 0, 0], [0,1,0,0]])):
    """Instantiates the drive template. Units should be SI. Free speed 
    should be radians per second, gear ratio should be greater than one, 
    stall current, free current, stall torque just refer to your motor specs for one
    of that type which should be found on Vex Motors. The dt refers to the robot's
    loop speed which as of 2018, is 5 ms (so dt = 0.005), and name refers to what
    you want to call the subsystem. Drivebase radius is a fudge factor for the 
    effects of the drivetrain on each side, a reasonable approximation is half
    the length of the drivetrain base."""
    state_space_controller.__init__(self, dt, name)
    #Input values
    self.N = num_stages_per_side
    self.n = num_motors_per_side
    self.I_stall = stall_current * self.n
    self.I_free = free_current * self.n
    self.tau_stall = stall_torque * self.n
    self.omega_free = free_speed
    self.J = moment_of_inertia
    self.G = gear_ratio_per_side
    self.r_w = wheel_radius
    self.r_b = drivebase_radius
    self.m = mass
    
    #Derived Values
    #efficiency
    self.mu = 0.95 ** self.N 
    #resistance
    self.R = 12.0 / self.I_stall
    self.k_t = self.I_stall / self.tau_stall
    self.k_v = (12.0 - self.I_free * self.R) / self.omega_free
    self.gamma_1 = (1.0 / self.m + self.r_b * self.r_b / self.J)#just placeholder constant so I don't retype all the variables
    self.gamma_2 = (1.0 / self.m - self.r_b * self.r_b / self.J)
    #coefficient of velocity
    self.k_vel = -self.k_v * self.G * self.G / (self.k_t * self.R * self.r_w * self.r_w)
    #coefficient of input
    self.k_u = self.mu * self.G / (self.k_t * self.R * self.r_w)
    
    #State is [[pos_left],[pos_right],[vel_left],[vel_right]]
    self.A_c = numpy.matrix([[0, 0, 1.0, 0],
                             [0, 0, 0, 1.0],
                             [0, 0, self.gamma_1 * self.k_vel, self.gamma_2 * self.k_vel],
                             [0 ,0, self.gamma_2 * self.k_vel, self.gamma_1 * self.k_vel]])
    self.B_c = numpy.matrix([[0, 0],
                             [0, 0],
                             [self.gamma_1 * self.k_u, self.gamma_2 * self.k_u],
                             [self.gamma_2 * self.k_u, self.gamma_1 * self.k_u]])
    
    self.C = C
    self.D = numpy.matrix([[0, 0],
                           [0, 0]])
    
    #converts continuous to discrete matrices
    self.A_d, self.B_d = controls.c2d(self.A_c,self.B_c,self.dt)
    
    self.minU = numpy.matrix([[-12.0], [-12.0]])
    self.maxU = numpy.matrix([[12.0], [12.0]])
    
    self.initialize_state() 
    
  def run_test(self, initial_X, goal, show_graph=True, use_observer=True, feet = True, iterations=4000):
    self.X = initial_X
    self.X_hat = self.X
    if use_observer:
      self.X_hat = initial_X + 0.03
    t = []
    x_l = []
    x_r = []
    v_l = []
    v_r = []
    x_l_hat = []
    x_r_hat = []
    v_l_hat = []
    v_r_hat = []
    u_l = []
    u_r = []
      
  #  sep_plot_gain - 100.0
    
    if feet:
      unchanged_goal = goal * 1.0 / 3.28084
    else:
      unchanged_goal = goal    
    
    for i in range(iterations):
      if not use_observer:
        self.X_hat = self.X
      if use_observer:
        if feet:
          x_l_hat.append(self.X_hat[0,0] * 3.28084)
          x_r_hat.append(self.X_hat[1,0] * 3.28084)
          v_l_hat.append(self.X_hat[2,0] * 3.28084)
          v_r_hat.append(self.X_hat[3,0] * 3.28084)
        else:
          x_l_hat.append(self.X_hat[0,0])
          x_r_hat.append(self.X_hat[1,0])
          v_l_hat.append(self.X_hat[2,0])
          v_r_hat.append(self.X_hat[3,0])
      U = self.K * (unchanged_goal - self.X_hat)
      U = numpy.clip(U, self.minU, self.maxU)
      if feet:
        x_l.append(self.X[0,0] * 3.28084)
        x_r.append(self.X[1,0] * 3.28084)
        v_l.append(self.X[2,0] * 3.28084)
        v_r.append(self.X[3,0] * 3.28084)
      else:
        x_l.append(self.X[0,0])
        x_r.append(self.X[1,0])
        v_l.append(self.X[2,0])
        v_r.append(self.X[3,0])
      if use_observer:
        self.predict_observer(U)
      self.update(U)
      if use_observer:
        self.correct_observer(U)
      t.append(i * self.dt)
      u_l.append(U[0,0])
      u_r.append(U[1,0])
      
    if show_graph:
      fig_size = [12, 9]
      pyplot.rcParams["figure.figsize"] = fig_size
      pyplot.subplot(2, 1, 1)
      pyplot.plot(t, x_l, label='left dist')
      pyplot.plot(t, x_r, label='right dist')
      pyplot.plot(t, v_l, label='left velocity')
      pyplot.plot(t, v_r, label='right velocity')
      print("Final State: ")
      if feet:
        print(self.X * 3.28084)
      else:
        print(self.X)
      print("\n")
      if use_observer:
        pyplot.plot(t, x_l_hat, label='x_hat left dist')
        pyplot.plot(t, x_r_hat, label='x_hat right dist')
        pyplot.plot(t, v_l_hat, label='x_hat left velocity')
        pyplot.plot(t, v_r_hat, label='x_hat right velocity')
        print("Final Estimated State: ")
        if feet:
          print(self.X_hat * 3.28084)
        else:
          print(self.X_hat)
      print("\n")
      pyplot.legend()
    
      pyplot.subplot(2, 1, 2)
      pyplot.plot(t, u_l, label='left voltage')
      pyplot.plot(t, u_r, label='right voltage')
      pyplot.legend()
      pyplot.show()
    
  def run_pid_test(self, constants, initial_X, goal, show_graph=True, feet = True, iterations=4000):
    """This method runs a PID controller test on the system. Constants should be
    an array with P, I, D constants in that order. deg refers to whether or not
    you want to convert to feet on the graph, it will be in meters if set to
    false. Iterations means how many loops to run. For a dt = 0.005, 4000 iterations
    is 20 seconds. A different dt will create a different length test for a given
    number of iterations. Smaller dt run more precisely, but consume more resources.
    Larger dt run with less resources, but are less accurate."""
    self.X = initial_X
    lcontroller = pid_controller.PIDController(constants[0], constants[1], constants[2], self.dt, goal[0,0])
    rcontroller = pid_controller.PIDController(constants[0], constants[1], constants[2], self.dt, goal[1,0])
    t = []
    x_l = []
    x_r = []
    v_l = []
    v_r = []

    u_l = []
    u_r = []
    
    for i in range(iterations):
      if feet:
        U = numpy.asmatrix([[lcontroller.Update(self.X[0,0]* 3.28084)], [rcontroller.Update(self.X[1,0]* 3.28084)]])
      else:
        U = numpy.asmatrix([[lcontroller.Update(self.X[0,0])], [rcontroller.Update(self.X[1,0])]])
      U = numpy.clip(U, self.minU, self.maxU)
      if feet:
        x_l.append(self.X[0,0] * 3.28084)
        x_r.append(self.X[1,0] * 3.28084)
        v_l.append(self.X[2,0] * 3.28084)
        v_r.append(self.X[3,0] * 3.28084)
      else:
        x_l.append(self.X[0,0])
        x_r.append(self.X[1,0])
        v_l.append(self.X[2,0])
        v_r.append(self.X[3,0])
      self.update(U)
      t.append(i * self.dt)
      u_l.append(U[0,0])
      u_r.append(U[1,0])
      
    if show_graph:
      fig_size = [12, 9]
      pyplot.rcParams["figure.figsize"] = fig_size
      pyplot.subplot(2, 1, 1)
      pyplot.plot(t, x_l, label='left dist')
      pyplot.plot(t, x_r, label='right dist')
      pyplot.plot(t, v_l, label='left velocity')
      pyplot.plot(t, v_r, label='right velocity')
      print("Final State: ")
      if feet:
        print(self.X * 3.28084)
      else:
        print(self.X)
      print("\n")
      #pyplot.plot(t, error, label='error')
      pyplot.legend()
    
      pyplot.subplot(2, 1, 2)
      pyplot.plot(t, u_l, label='left voltage')
      pyplot.plot(t, u_r, label='right voltage')
      pyplot.legend()
      pyplot.show()
      

  def run_custom_test(self, initial_X, goal, f, show_graph=True, use_observer=True, feet = True, iterations=4000):
    """This method runs a custom test on the system. f should be a user defined
    function. feet refers to whether or not
    you want to convert to feet on the graph, it will be in meters if set to
    false. Iterations means how many loops to run. For a dt = 0.005, 4000 iterations
    is 20 seconds. A different dt will create a different length test for a given
    number of iterations. Smaller dt run more precisely, but consume more resources.
    Larger dt run with less resources, but are less accurate. For Fabrication 
    and CAD, use_observer should be set to False"""

    self.X = initial_X
    self.X_hat = self.X
    if use_observer:
      self.X_hat = initial_X + 0.03
    t = []
    x_l = []
    x_r = []
    v_l = []
    v_r = []
    x_l_hat = []
    x_r_hat = []
    v_l_hat = []
    v_r_hat = []
    u_l = []
    u_r = []
    torque = []
      
  #  sep_plot_gain - 100.0
    
    if feet:
      unchanged_goal = goal * 1.0 / 3.28084
    else:
      unchanged_goal = goal    
    
    for i in range(iterations):
      if not use_observer:
        self.X_hat = self.X
      if use_observer:
        if feet:
          x_l_hat.append(self.X_hat[0,0] * 3.28084)
          x_r_hat.append(self.X_hat[1,0] * 3.28084)
          v_l_hat.append(self.X_hat[2,0] * 3.28084)
          v_r_hat.append(self.X_hat[3,0] * 3.28084)
        else:
          x_l_hat.append(self.X_hat[0,0])
          x_r_hat.append(self.X_hat[1,0])
          v_l_hat.append(self.X_hat[2,0])
          v_r_hat.append(self.X_hat[3,0])
      U = f(unchanged_goal - self.X_hat)
      U = numpy.clip(U, self.minU, self.maxU)
      torque.append(U[0,0]- self.k_v * self.G * self.X[2,0] / (math.pi * self.r_w * 2.0))
      if feet:
        x_l.append(self.X[0,0] * 3.28084)
        x_r.append(self.X[1,0] * 3.28084)
        v_l.append(self.X[2,0] * 3.28084)
        v_r.append(self.X[3,0] * 3.28084)
      else:
        x_l.append(self.X[0,0])
        x_r.append(self.X[1,0])
        v_l.append(self.X[2,0])
        v_r.append(self.X[3,0])
      if use_observer:
        self.predict_observer(U)
      #self.X = self.ddynamics(self.X, torque)
      #self.Y = self.C * self.X + self.D * U
      self.update(U)
      if use_observer:
        self.correct_observer(U)
      t.append(i * self.dt)
      u_l.append(U[0,0])
      u_r.append(U[1,0])
      
    if show_graph:
      fig_size = [12, 9]
      pyplot.rcParams["figure.figsize"] = fig_size
      pyplot.subplot(2, 1, 1)
      pyplot.plot(t, x_l, label='left dist')
      pyplot.plot(t, x_r, label='right dist')
      pyplot.plot(t, v_l, label='left velocity')
      pyplot.plot(t, v_r, label='right velocity')
      pyplot.plot(t, torque, label='torque')
      print("Final State: ")
      if feet:
        print(self.X * 3.28084)
      else:
        print(self.X)
      print("\n")
      if use_observer:
        pyplot.plot(t, x_l_hat, label='x_hat left dist')
        pyplot.plot(t, x_r_hat, label='x_hat right dist')
        pyplot.plot(t, v_l_hat, label='x_hat left velocity')
        pyplot.plot(t, v_r_hat, label='x_hat right velocity')
        print("Final Estimated State: ")
        if feet:
          print(self.X_hat * 3.28084)
        else:
          print(self.X_hat)
        print("\n")
      pyplot.legend()
    
      pyplot.subplot(2, 1, 2)
      pyplot.plot(t, u_l, label='left voltage')
      pyplot.plot(t, u_r, label='right voltage')
      pyplot.legend()
      pyplot.show()
      
  def set_K(self, K):
    self.K = numpy.asmatrix(K)

  def set_L(self, L):
    self.L = numpy.asmatrix(L)

  def get_stats(self, feet = True):
    """This method will get you the theoretical maximums of the system."""
    self.max_vel = -self.k_u * 12.0 / self.k_vel
    self.max_acc = self.k_u * 12.0
    self.max_jerk = -self.k_u * self.k_vel * 12.0 / (self.m ** 2.0)
    if feet:
      self.max_vel *= 3.28084
      self.max_acc *= 3.28084
      self.max_jerk *= 3.28084
    print("Maximum Velocity: " + str(self.max_vel) + "\n")
    print("Maximum Acceleration: " + str(self.max_acc) + "\n")    
    print("Maximum Jerk: " + str(self.max_jerk) + "\n")    

