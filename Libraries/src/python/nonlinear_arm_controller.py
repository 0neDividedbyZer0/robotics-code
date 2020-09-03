# -*- coding: utf-8 -*-
"""
Created on Sun Oct 14 00:27:56 2018

@author: Maverick1
"""

import numpy
from matplotlib import pyplot
import controls
import math
import pylab
from scipy.integrate import odeint
import matplotlib.animation as animation

class NonlinearArmController():
    def __init__(self, k, dt, setpoint):
        self.t = 0.0
        self.k = k
        self.dt = dt
        self.setpoint = setpoint
        self.free_speed = 18730.0 * 2.0 * math.pi / 60.0 #radians per second
        self.free_current = 0.7  #amps
        self.stall_torque = 0.71 # newton meters
        self.stall_current = 134.0
        self.resistence = 12.0 / self.stall_current
        self.k_t = self.stall_torque / self.stall_current 
        self.k_v = (12.0 - self.free_current * self.resistence) / self.free_speed
        self.gear_ratio = 833.33
        self.mass = 10.0 * 0.454 #kilograms
        self.length = 17.0 * 0.0254 #meters
        self.efficiency = 0.65;
        self.gravity = 9.8 #m/s^2
        
        self.coeff_theta_dot = self.k_v * self.gear_ratio / self.efficiency
        self.coeff_E_tilde_theta_dot = -self.k * self.resistence / (self.gear_ratio * self.k_t 
                                                           *self.efficiency)
        self.coeff_sin = -2.0 * self.mass * self.gravity * self.length * self.resistence / (
                self.gear_ratio * self.k_t * self.efficiency)
        
        self.a = -self.k * self.resistence / (self.gear_ratio * self.k_t * self.efficiency)
        self.b = self.k_v * self.gear_ratio /self.efficiency
        
        
    def setSetpoint(self, setpoint):
        self.setpoint = setpoint
        
    def update(self, theta, theta_dot):
        K = 0.5 * self.mass * self.length * self.length * theta_dot * theta_dot
        U = self.mass * self.gravity * self.length * math.cos(theta)
        E = K + U
        E_tilde = E - self.setpoint
        #return self.coeff_theta_dot * theta_dot + self.coeff_E_tilde_theta_dot * theta_dot * E_tilde + self.coeff_sin * math.sin(theta)
        return self.a * theta_dot * E_tilde + self.b * theta_dot
    def dynamics(self, state, t):
        x = state[0]
        dx = state[1]
        u = min(max(self.update(x, dx),-12.0),12.0)
        tau = 0.0#(self.efficiency * self.gear_ratio * self.k_t / self.resistence) * u - self.gear_ratio * self.gear_ratio * self.k_t * self.k_v * dx / self.resistence
        #tau = self.update(x, dx)
        ddx = (self.gravity / self.length) * math.sin(x) + tau / (self.mass * self.length * self.length)
        
        return [dx, ddx]
    
    def position(self, theta):
        x = (0,self.length *math.sin(theta))
        y = (0,self.length *math.cos(theta))
        
        return (x,y)
    
    def ddynamics(self, state, dt):
        delta_x1 = state[0] + dt * state[1] - dt * dt * self.gravity / (2.0 *self.length) * math.sin(state[0])
        delta_x2 = state[1]
        return [delta_x1, delta_x2]
     
    def init(self):
        """initialize animation"""
        self.line.set_data([], [])
        self.time_text.set_text('')
        return self.line, self.time_text
    
    def animate(self, i):
        """perform animation step"""
        
        self.line.set_data(*self.position(self.state[i][0]))
        self.t += self.dt * 0.1
        self.time_text.set_text('time = %.1f' % self.t)
        return self.line, self.time_text
    
    def simulate(self):
        state0 = [1.0, -20]
        t = pylab.arange(0.0, 10.0, 0.1 * 0.005)
    
        self.state = odeint(self.dynamics, state0, t)
        #pylab.plot(t, self.state)
        theta = []
        omega = []
        dis_t = []
        state = [1.0, -20.0]
        for i in range(2000):
            state = self.ddynamics(state, 0.005)
            theta.append(state[0])
            omega.append(state[1])
            dis_t.append(i * 0.005)
        #fig_size = [12, 9]
        #pyplot.rcParams["figure.figsize"] = fig_size
        #pyplot.subplot(2, 1, 1)
        pyplot.plot(t, theta, label='angle')
        print(state)
        print(omega)
        #pyplot.plot(t, omega, label='angular velocity')
        
        #return self.state
    
    def setup_graph(self):
        self.fig = pyplot.figure()
        self.ax = self.fig.add_subplot(111, aspect='equal', autoscale_on=False,
                             xlim=(-2, 2), ylim=(-2, 2))
        self.ax.grid()
        
        self.line, = self.ax.plot([], [], 'o-', lw=2)
        self.time_text = self.ax.text(0.02, 0.95, '', transform=self.ax.transAxes)
        
    def visualize(self):
        self.setup_graph()
        
        from time import time
        t0 = time()
        self.animate(0)
        t1 = time()
        interval = 20000 * self.dt*0.1 - (t1 - t0)
        
        ani = animation.FuncAnimation(self.fig, self.animate, frames=300,
                                      interval=interval, blit=True, init_func=self.init)
        pyplot.show()
    
def main(controller):
    controller.simulate()
    #controller.visualize()
        
controller = NonlinearArmController(0.001, 0.005, 9.8 * 10.0 * 0.454 * 17.0 * 0.0254)

main(controller)

        
        
        