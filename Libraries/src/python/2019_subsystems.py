# -*- coding: utf-8 -*-
"""
Created on Tue Jan  8 17:47:04 2019

@author: Maverick1
"""

import math
import controls
import numpy
from linear_actuator import linear_actuator
from elevator import elevator
from drive import drive

#Basically, just send in 12 volts and see the effects :D
def max_func(x = numpy.asmatrix([[0]])):
  return numpy.asmatrix([[12.0]])
#same but for drive

def full_volts(x = numpy.asmatrix([[0],[0]])):

  #if(x[0,0] >= 5.0 and x[1, 0] >= 5.0):
      return numpy.asmatrix([[12.0],[ 12.0]])
  #else:
   #   return numpy.asmatrix([[-12.0],[-12.0]])
  
def calculate_load_torque(load_force, diameter, pitch, mu): #please use si
    l = 1.0 / pitch
    return 0.5 * load_force * diameter * (l + math.pi * mu * diameter) / (math.pi * diameter - mu * l)
def calculate_ideal_gear_ratio(load_torque, ideal_torque):
    G = load_torque / ideal_torque
    return G
  
def main():
    rod_weight = 4.0 * 0.293 * 0.453592
    radius = 3.0 / 16.0 * 0.0254
    pitch = 16.0/ (1 * 0.0254)
    load = calculate_load_torque(40. * 4.4482, 2.0 * radius, pitch, 0.19)
    test = linear_actuator(0.005, "test", 1.0, 1.0, 105.0, 1.8, 2.6, 5676 * 2.0 * math.pi / 60.0,
             0.5 * rod_weight * radius * radius, 1.0, pitch, load)
    
    #test.run_custom_test(numpy.matrix([[0.0],[0]]), numpy.matrix([[24.0],[0]]), max_func, True, False, True, 800)
    
    #test.get_stats()
    best_G = calculate_ideal_gear_ratio(load, 1.3)
    print(best_G)
    #Neo @ 1:3 overdrive running at 10 volts works best
    
    
    drive_gear_ratio = 7.083 * 24.0 / 60.0 * 64.0 / 20.0;#1.13333#5.10#50.0 /34.0 2.16 ratio spread 54 : 30 
    j = 6.0
    
    #2778 2000 2907 3269 3211 3099 3287 3573 2705
    
    #drivetrain = drive(0.005, "drivetrain", 4.0, 2.0, 131.0, 2.7, 2.41, 5330 * 2.0 * math.pi / 60.0,
    #              drive_gear_ratio, j, 3.0 * 0.0254, 28.0 * 0.0254 * 0.5, 154.0 * 0.454)
    #radius base might be 25.5 * 0.0254 * 0.5, but not likely, more like 0.45
    drivetrain = drive(0.005, "drivetrain", 3.0, 2.0, 131.0, 2.7, 2.41, 5330 * 2.0 * math.pi / 60.0,
                     drive_gear_ratio, j, 3.0 * 0.0254, 25.5 * 0.0254 * 0.5 , (125.0) * 0.454)
    #drivetrain.run_pid_test([1.0, 0.0,0],numpy.matrix([[0.0],[0],[0],[0]]), numpy.matrix([[55.0],[55.0], [0.0],[0.0]]), True, True, 4000)
    #drivetrain.run_custom_test(numpy.matrix([[0.0],[0],[0],[0]]), numpy.matrix([[12.0],[12.0], [0.0],[0.0]]), full_volts, True, False, True, 2000)
    drivetrain.get_stats(True)
    
    driveQ_c = numpy.matrix([[1.0 / (0.14)**2.0 , 0, 0, 0],
                             [0, 1.0 / (0.14)**2.0, 0, 0],
                             [0, 0, 1.0 / (1.0)**2.0, 0],
                             [0, 0, 0, 1.0 / (1.0)**2.0]])
    
    driveR_c = numpy.matrix([[0.0001**2, 0.0],
                             [0.0, 0.0001 **2]])
                           # [0 , 1.0 / (.1 * 0.0254)**2]])
                           
    driveA_d, driveB_d, driveQ_d, driveR_d = controls.c2d(drivetrain.A_c, drivetrain.B_c, 0.005, driveQ_c, driveR_c)
    
    driveK = controls.dlqr(driveA_d, driveB_d, driveQ_d, driveR_d)
    
    print(driveK)
    
    
    elev = elevator(0.005, "elev", 2.0, 2.0, 134.0, 0.7, 0.71, 18730.0 * 2.0 * math.pi / 60.0,
                  63.0, 3.0 * (0.8755 - 0.16) * 0.0254, 18.8* 0.454)#11.59)
    
    #elev.export("../python")
    
    elevQ_c = numpy.matrix([[1.0 / (0.0254 * 0.01)**2.0 , 0],
                            [0 , 1.0 / (.1 * 0.0254)**2]])
    elevR_c = numpy.matrix([[1.0 / 13.0**2]])
    
    elevA_d, elevB_d, elevQ_d, elevR_d = controls.c2d(elev.A_c, elev.B_c, 0.005, elevQ_c, elevR_c)
    
    elevK = controls.dlqr(elevA_d, elevB_d, elevQ_d, elevR_d)
    
    elevL = controls.dkalman(elevA_d, elevB_d, elevQ_d, elevR_d)
    
    #print(elevK)
    
  
    elev.run_test([[0.],[0.]],[[0.0],[60.0]])
    #elev.run_pid_test([elevK[0,0] * 0.0254, 0.0,elevK[0,1] * 0.0254],numpy.matrix([[0.0],[0]]), numpy.matrix([[72.0],[0]]), True, True, 400)
    #elev.run_custom_test(numpy.matrix([[0.0],[0]]), numpy.matrix([[60.0],[0]]), max_func, True, False, True, 400)
    #elev.get_stats()
if __name__ == "__main__":
  main() 