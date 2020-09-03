# -*- coding: utf-8 -*-
"""
Created on Fri Apr 20 17:07:30 2018

@author: Maverick Zhang
"""

import numpy
from matplotlib import pyplot
import math
import cmath
import pid_controller
import trapezoidal_profile
import state_space_controller as ss
import controls
import matplotlib.animation as animation

class Arm(ss.StateSpaceController):
  def __init__(self, dt, name):
    ss.StateSpaceController.__init__(self, dt)
    self.origin = (0,0)
    self.time_elapsed = 0.0
    self.has_cube = False
    self.free_speed = 18730.0 * 2.0 * math.pi / 60.0 #radians per second
    self.free_current = 0.7  #amps
    self.stall_torque = 0.71 # newton meters
    self.stall_current = 134.0
    self.resistence = 12.0 / self.stall_current
    self.k_t = self.stall_torque / self.stall_current 
    self.k_v = (12.0 - self.free_current * self.resistence) / self.free_speed
    self.gear_ratio = 833.33
    self.mass = 10.0 * 0.454 #kilograms
    if(self.has_cube):
        self.mass += 1.59
    self.radius = 17.0 * 0.0254 #meters
    self.moment_of_inertia = self.mass * self.radius * self.radius#10.0 #this is speculative
    self.efficiency = 0.65;
    
    self.a = (-self.k_t * self.k_v * self.gear_ratio * self.gear_ratio) / (self.moment_of_inertia * self.resistence)
    self.b = (self.efficiency * self.k_t * self.gear_ratio) / (self.moment_of_inertia * self.resistence)
    
    #state is [angle,
    #          angular_velocity]
    #input is [voltage]
    self.A_c = numpy.matrix([[0 , 1],
                             [0 , self.a]])
    self.B_c = numpy.matrix([[0],
                             [self.b]])
    self.C = numpy.matrix([[1 , 0]])
    self.D = numpy.matrix([[0]])
    q_1 = 1.0/((math.pi / 90.0)**2)
    q_2 = 1.0/((math.pi / 90.0)**2)
    self.Q_c = numpy.matrix([[q_1, 0],
                             [0, q_2]])
    r_1 = 1.0 / (0.1**2)
    self.R_c = numpy.matrix([[r_1]])
    
    self.temp = controls.clqr(self.A_c,self.B_c,self.Q_c,self.R_c)
    
    print(controls.eig(self.A_c-self.B_c*self.temp))
    
    self.A_d, self.B_d, self.Q_d, self.R_d = controls.c2d(self.A_c,self.B_c,self.dt, self.Q_c,self.R_c)
    
    #s =  0,-295.41341225 are open loop poles
    #converting s to z is through e^(sT) where T is the discrete time step.
    #(0.995 , 0.874) these are when s = -1, and s = -27
    #tried with poles at 4.2, -295.41341225
    poles = (cmath.exp(0.005*(-27.0)),cmath.exp(0.005*(-28.0)))
    polesd = (-27.0+3.0j,-27.0-3.0j)
    self.K = controls.place(self.A_d, self.B_d, poles)
    
    self.Kff = controls.feedforwards(self.A_d, self.B_d, self.Q_d)
    
    #These should be chosen 10 times faster in the s domain, and then converted
    #to the z domain
    #(0.951, 0.259) these are when s = -10, s= -270
    self.L = controls.dkalman(self.A_d,self.C,self.Q_d,self.R_d)
    
    self.minU = numpy.matrix([[-12.0]])
    self.maxU = numpy.matrix([[12.0]])
    
    self.initialize_state() 
    
  def VtoT(self, X, voltage):
    torque = (self.efficiency * voltage - self.k_v * self.gear_ratio * X[1,0] )/ (self.resistence / (self.gear_ratio * self.k_t))
    return numpy.asmatrix([[torque]])

  def ddynamics(self, X, U):
    delta_x1 = X[0,0] + self.dt * X[1,0] - self.dt * self.dt * 9.8 / (2.0 *self.radius) * math.sin(X[0,0])
    delta_x2 = X[1,0] + self.dt / (self.mass * self.radius * self.radius) * U[0,0]
    return numpy.asmatrix([[delta_x1], 
                           [delta_x2]])

  def run_test(self, initial_X, goal, show_graph=True, use_observer=True, iterations=4000):
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
    
    unchanged_goal = goal
    
    for i in range(iterations):
      if not use_observer:
        self.X_hat = self.X
      if use_observer:
        theta_hat.append(self.X_hat[0,0])
        omega_hat.append(self.X_hat[1,0])
      U = self.K * (unchanged_goal - self.X_hat)
      U = numpy.clip(U, self.minU, self.maxU)
      #torque = self.VtoT(self.X, U[0,0])
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
      u.append(U[0,0])
      
    if show_graph:
      fig_size = [12, 9]
      pyplot.rcParams["figure.figsize"] = fig_size
      pyplot.subplot(2, 1, 1)
      pyplot.plot(t, theta, label='angle')
      pyplot.plot(t, omega, label='angular velocity')
      if use_observer:
        pyplot.plot(t, theta_hat, label='x_hat angle')
        pyplot.plot(t, omega_hat, label='x_hat angular velocity')
      pyplot.legend()
    
      pyplot.subplot(2, 1, 2)
      pyplot.plot(t, u, label='voltage')
      pyplot.legend()
      pyplot.show()
      
  def run_pid_test(self, constants, initial_X, goal, show_graph=True, iterations=4000):
    self.X = initial_X
    controller = pid_controller.PIDController(constants[0], constants[1], constants[2], self.dt, self.r2d(goal[0,0]))
    t = []
    theta = []
    omega = []
    error = []
    u = []
    
    for i in range(iterations):
      U = [controller.Update(self.r2d(self.X[0,0]))]
      U = numpy.clip(U, self.minU, self.maxU)
      theta.append(self.r2d(self.X[0,0]))
      omega.append(self.r2d(self.X[1,0]))
      error.append(self.r2d(goal[0,0]-self.X[0,0]))
      self.update(U)
      t.append(i * self.dt)
      u.append(U[0,0])
      
    if show_graph:
      fig_size = [12, 9]
      pyplot.rcParams["figure.figsize"] = fig_size
      pyplot.subplot(2, 1, 1)
      pyplot.plot(t, theta, label='angle')
      pyplot.plot(t, omega, label='angular velocity')
      #pyplot.plot(t, error, label='error')
      pyplot.legend()
    
      pyplot.subplot(2, 1, 2)
      pyplot.plot(t, u, label='voltage')
      pyplot.legend()
      pyplot.show()
      pyplot.savefig("foo.png")
      
  def run_profiled_test(self, profile, constants, initial_X, show_graph=True, iterations=4000):
      self.X = initial_X;
      trap_profile = trapezoidal_profile.TrapezoidalMotionProfile(profile[0], profile[1], profile[2])
      controller = pid_controller.PIDController(constants[0], constants[1], constants[2], self.dt, [trap_profile.distance(0), trap_profile.velocity(0)])
      t = []
      theta = []
      omega = []
      u = []
      profile_vel = []
    
      for i in range(iterations):
        controller.setSetpoint(trap_profile.distance(i * self.dt))
        U = [controller.Update(self.X[0,0]) +  (12.0 / profile[1]) * trap_profile.velocity(i * self.dt)]
        U = numpy.clip(U, self.minU, self.maxU)
        theta.append(self.X[0,0])
        omega.append(self.X[1,0])
        self.update(U)
        t.append(i * self.dt)
        u.append(U[0,0])
        profile_vel.append((12.0 / profile[1]) * trap_profile.velocity(i * self.dt))
      
      if show_graph:
        fig_size = [12, 9]
        pyplot.rcParams["figure.figsize"] = fig_size
        pyplot.subplot(2, 1, 1)
        pyplot.plot(t, theta, label='angle')
        pyplot.plot(t, omega, label='angular velocity')
        pyplot.legend()
    
        pyplot.subplot(2, 1, 2)
        pyplot.plot(t, u, label='voltage')
        pyplot.plot(t, profile_vel, label='profile velocity')
        pyplot.legend()
        pyplot.show()
        
  def r2d(self, radians):
    return radians * 180.0 / math.pi
        
  def run_accel_test(self, initial_X, show_graph=True, use_observer=False, iterations=4000):
    self.X = initial_X
    self.X_hat = self.X
    if use_observer:
      self.X_hat = initial_X + 0.03
    t = []
    theta = []
    omega = []
    alpha = []
    theta_hat = []
    omega_hat = []
    u = []
    
      
  #  sep_plot_gain - 100.0
    
    lastVel = 0.0
    for i in range(iterations):
      if not use_observer:
        self.X_hat = self.X
      if use_observer:
        theta_hat.append(self.X_hat[0,0])
        omega_hat.append(self.X_hat[1,0])
      U = numpy.matrix([[12.0]])
      U = numpy.clip(U, self.minU, self.maxU)
      theta.append(self.X[0,0])
      self.X[0,0]
      omega.append(self.X[1,0])
      alpha.append((self.X[1,0] - lastVel)/self.dt)
      lastVel = self.X[1,0]
      if use_observer:
        self.predict_observer(U)
      self.update(U)
      if use_observer:
        self.correct_observer(U)
      t.append(i * self.dt)
      u.append(U[0,0])
      
    print(alpha[1])
    print(self.a)
      
    if show_graph:
      fig_size = [12, 9]
      pyplot.rcParams["figure.figsize"] = fig_size
      pyplot.subplot(2, 1, 1)
      pyplot.plot(t, theta, label='angle')
      pyplot.plot(t, omega, label='angular velocity')
      #pyplot.plot(t, alpha, label='angular acceleration')
      if use_observer:
        pyplot.plot(t, theta_hat, label='x_hat angle')
        pyplot.plot(t, omega_hat, label='x_hat angular velocity')
      pyplot.legend()
    
      pyplot.subplot(2, 1, 2)
      pyplot.plot(t, u, label='voltage')
      pyplot.legend()
      pyplot.show()
      
  def run_ss_profiled_test(self, profile, constants, initial_X, use_observer = True, show_graph=True, iterations=4000):
      self.X = initial_X
      prev_setpoint = self.X
      if use_observer:
        self.X_hat = initial_X + 0.03
        prev_setpoint = self.X_hat
      trap_profile = trapezoidal_profile.TrapezoidalMotionProfile(profile[0], profile[1], profile[2])
      t = []
      theta = []
      omega = []
      theta_hat = []
      omega_hat = []
      u = []
      profile_vel = []
    
      for i in range(iterations):
        if not use_observer:
          self.X_hat = self.X
        if use_observer:
          theta_hat.append(self.X_hat[0,0])
          omega_hat.append(self.X_hat[1,0])
        r = [[trap_profile.distance(i * self.dt)],[trap_profile.velocity(i * self.dt)]]
        U = self.K*(prev_setpoint - self.X_hat) + self.Kff*(r-self.A_d * prev_setpoint)
        U = numpy.clip(U, self.minU, self.maxU)
        if use_observer:
          self.predict_observer(U)
        self.update(U)
        if use_observer:
          self.correct_observer(U)
        theta.append(self.X[0,0])
        omega.append(self.X[1,0])
        t.append(i * self.dt)
        u.append(U[0,0])
        profile_vel.append((12.0 / profile[1]) * trap_profile.velocity(i * self.dt))
        prev_setpoint = r
      
      if show_graph:
        fig_size = [12, 9]
        pyplot.rcParams["figure.figsize"] = fig_size
        pyplot.subplot(2, 1, 1)
        pyplot.plot(t, theta, label='angle')
        pyplot.plot(t, omega, label='angular velocity')
        if use_observer:
          pyplot.plot(t, theta_hat, label='x_hat angle')
          pyplot.plot(t, omega_hat, label='x_hat angular velocity')
        pyplot.legend()
    
        pyplot.subplot(2, 1, 2)
        pyplot.plot(t, u, label='voltage')
        pyplot.plot(t, profile_vel, label='profile velocity')
        pyplot.legend()
        pyplot.show()

  def position(self):
    x = (self.origin[0],self.radius *math.sin(self.X[0,0]))
    y = (self.origin[1],self.radius *math.cos(self.X[0,0]))
    
    return (x,y)

  def animate_step(self, unchanged_goal):
    U = self.K * (unchanged_goal - self.X)
    U = numpy.clip(U, self.minU, self.maxU)
    self.update(U)
    self.time_elapsed += self.dt

def main():
  arm.run_test(numpy.matrix([[0.0],[0]]), numpy.matrix([[math.pi / 2.0],[0]]), True, False, 100)
  #arm.run_pid_test([15, 3, 100000],numpy.matrix([[0.0],[0]]), numpy.matrix([[math.pi / 4.0],[0]]), True, 400)
  #arm.run_profiled_test([math.pi/2.0, 1.538, 36.151],[30, 0.0, 0.0],numpy.matrix([[0.0],[0]]), True, 300)
  #arm.run_accel_test(numpy.matrix([[0.0],[0]]), True, False, 400)
  #arm.run_ss_profiled_test([math.pi/2.0, 1.538, 36.151],[0, 0.0, 0.0],numpy.matrix([[0.0],[0]]), True, True, 300)
  arm.export("C:/Users/Maverick1/Documents/frc1671-robot-code/Doc14_254/src/python/arm")
   

arm = Arm(0.005)

fig = pyplot.figure()
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                     xlim=(-2, 2), ylim=(-2, 2))
ax.grid()

line, = ax.plot([], [], 'o-', lw=2)
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

def init():
    """initialize animation"""
    line.set_data([], [])
    time_text.set_text('')
    return line, time_text

def animate(i):
    """perform animation step"""
    global pendulum, dt
    arm.animate_step(numpy.matrix([[100*math.pi/2.0],[0.0]]))
    
    line.set_data(*arm.position())
    time_text.set_text('time = %.1f' % arm.time_elapsed)
    return line, time_text

# choose the interval based on dt and the time to animate one step
from time import time
t0 = time()
animate(0)
t1 = time()
interval = 1000 * arm.dt - (t1 - t0)

ani = animation.FuncAnimation(fig, animate, frames=300,
                              interval=interval, blit=True, init_func=init)

# save the animation as an mp4.  This requires ffmpeg or mencoder to be
# installed.  The extra_args ensure that the x264 codec is used, so that
# the video can be embedded in html5.  You may need to adjust this for
# your system: for more information, see
# http://matplotlib.sourceforge.net/api/animation_api.html
#ani.save('double_pendulum.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
pyplot.show()

if __name__ == "__main__":
  main()  
      
      
      
      