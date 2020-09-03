# -*- coding: utf-8 -*-
"""
Created on Mon Jan  7 22:33:28 2019

@author: Maverick1
"""
import math
import controls
import numpy
from scipy.integrate import odeint
import matplotlib.pyplot as plt

def liftoff(mass, gravity, horizontal_dist_to_COM_front, horizontal_dist_to_COM_rear,
            COM_height, accel_force, brake_force):
    F_resultant = accel_force - brake_force
    wheelbase = horizontal_dist_to_COM_front + horizontal_dist_to_COM_rear
    N_1 = (horizontal_dist_to_COM_rear * mass * gravity - COM_height * F_resultant) / wheelbase
    N_2 = (horizontal_dist_to_COM_front * mass * gravity + COM_height * F_resultant) / wheelbase
    return (N_1, N_2)

"""I'm assuming degrees you monster"""
def transform(angle_of_wheel_drop, coordinate_of_COM_rel_to_center_of_drivebase, wheel_drop, aligned_wheel_gap, wheel_radius):
    l = math.hypot(wheel_drop, aligned_wheel_gap)
    alpha = math.pi / 180.0 * angle_of_wheel_drop
    R = numpy.matrix([[math.cos(alpha), -math.sin(alpha)],
                      [math.sin(alpha), math.cos(alpha)]])
    translation = numpy.matrix([[0.0],
                                [wheel_drop]])
    r = coordinate_of_COM_rel_to_center_of_drivebase + translation
    result = R * r
    x = result[0,0]
    y = result[1,0] + wheel_radius
    if(alpha > 0):
        return (-x, l + x, y)
    else:
        return (l - x, x, y)
    
def dUdt(U, t):
    g = 9.8
    l = 1.0
    return [U[1], g / l * numpy.sin(U[0])]
    
    
def simulate(U_0, t):
    time = numpy.linspace(0.0, t)
    U = odeint(dUdt, U_0, time)
    ys = U[:,0]
    
    plt.plot(t,ys)
    plt.xlabel('time')
    plt.ylabel('y(t)')
    plt.show()

def main():
    simulate([0,1], 20.0)
    
    wheel_drop = 0.125 * 0.0254
    aligned_wheel_gap = 12.0 * 0.0254
    theta = 180.0 / math.pi * math.atan(wheel_drop / aligned_wheel_gap)
    print(theta)
    coord = numpy.matrix([[0.33],
                          [.78]])
    (a, b, c) = transform(-theta, coord, wheel_drop, aligned_wheel_gap, 3.0 * 0.0254)
    mass = 120.0 * 0.454
    (u, v) = liftoff(mass, 9.80, a, b, c, 0.0, 1.15)
    #complete - to positive brake = 
    print(u,v)

if __name__ == "__main__":
  main()  