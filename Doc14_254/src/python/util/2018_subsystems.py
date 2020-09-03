# -*- coding: utf-8 -*-
"""
Created on Sat Nov  3 20:35:43 2018

@author: Maverick1
"""
import math
import controls
import numpy
from arm import arm
from elevator import elevator
from drive import drive
from shooter import shooter

#Basically, just send in 12 volts and see the effects :D
def max_func(x = numpy.asmatrix([[0]])):
  return numpy.asmatrix([[12.0]])

#same but for drive
def full_volts(x = numpy.asmatrix([[0],[0]])):
  return numpy.asmatrix([[12.0],[12.0]])

def main():
  wrist = arm(0.005, "wrist", 8.0, 1.0, 134.0, 0.7, 0.71, 18730.0 * 2.0 * math.pi / 60.0,
              1.0 / 3.0 * 10.0 * 0.454 * ((17.0 * 0.0254) ** 2), 833.33)
  """q_1 = 1.0/((math.pi / 90.0)**2)
  q_2 = 1.0/((math.pi / 90.0)**2)
  Q_c = numpy.matrix([[q_1, 0],
                      [0, q_2]])
  r_1 = 1.0 / (0.1**2)
  R_c = numpy.matrix([[r_1]])
  A_d, B_d, Q_d, R_d = controls.c2d(numpy.asmatrix(wrist.A_c), numpy.asmatrix(wrist.B_c),0.005,  Q_c, R_c)
  K = controls.place(A_d, B_d, (0.9, 0.003))
  L = controls.dkalman(A_d, wrist.C, Q_d, R_d)
  wrist.set_K(K)
  wrist.set_L(L)"""
  
  #wrist.run_test(numpy.matrix([[0.0],[0]]), numpy.matrix([[90.0],[0.0]]), True, False, True, 400)
  #wrist.run_pid_test([5.23, 0.0,0.0],numpy.matrix([[0.0],[0]]), numpy.matrix([[90.0],[0]]), True, True, 400)
  #wrist.run_custom_test(numpy.matrix([[0.0],[0]]), numpy.matrix([[90.0],[0]]), max_func, True, False, True, 400)
  wrist.get_stats()
  
  elev = elevator(0.005, "elev", 4.0, 4.0, 134.0, 0.7, 0.71, 18730.0 * 2.0 * math.pi / 60.0,
                  40.62, 1.751 * 0.0254, 10.0* 0.454 + 2.5)#11.59)
  
  
  #elev.run_pid_test([1.2, 0.0,0.0],numpy.matrix([[0.0],[0]]), numpy.matrix([[60.0],[0]]), True, True, 400)
  #elev.run_custom_test(numpy.matrix([[0.0],[0]]), numpy.matrix([[60.0],[0]]), max_func, True, False, True, 400)
  elev.get_stats()
  
  drive_gear_ratio = (60.0 / 12.0) * (34.0 / 24.0) * (34.0/ 32.0)
  distribution_radius = 0.45
  mass = 154.0 * 0.454
  j = distribution_radius * mass
  print(drive_gear_ratio)
  drivetrain = drive(0.005, "drivetrain", 4.0, 3.0, 89.0, 3.0, 1.41, 5840 * 2.0 * math.pi / 60.0,
                drive_gear_ratio, j, 3.0 * 0.0254, 28.0 * 0.0254 * 0.5, 154.0 * 0.454)
  #drivetrain.run_pid_test([1.2, 0.0,.1],numpy.matrix([[0.0],[0],[0],[0]]), numpy.matrix([[12.0],[8.0], [0.0],[0.0]]), True, True, 4000)
  #drivetrain.run_custom_test(numpy.matrix([[0.0],[0],[0],[0]]), numpy.matrix([[12.0],[12.0], [0.0],[0.0]]), full_volts, True, False, True, 400)
  drivetrain.get_stats()
  
  intake = shooter(0.005, "intake", 4.0, 2.0, 134.0, 0.7, 0.71, 18730.0 * 2.0 * math.pi / 60.0,
                   0.5 * 0.0254 ** 2 * 10.0, 6.67)
  
  intake.run_pid_test([0.05, 0.07,0.0],numpy.matrix([[0.0]]), numpy.matrix([[2000.0]]), True, True, 400)
  #intake.run_custom_test(numpy.matrix([[0.0]]), numpy.matrix([[1000]]), max_func, True, False, True, 400)
  intake.get_stats()
if __name__ == "__main__":
  main()  