# -*- coding: utf-8 -*-
import math

"""
A simple trapezoidal motion profile class in python for simulation. Code is 
basically from 1678, but it's only because I didn't want to type in 
the variables again.
"""
__author__ = 'Maverick Zhang'

class trapezoidal_motion_profile(object):
    def __init__(self, distance, max_speed, max_acceleration):
        self.total_distance = distance
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration

        self.acceleration_time = self.max_speed / self.max_acceleration

        if self.acceleration_time * self.acceleration_time * self.max_acceleration > self.total_distance:
            self.acceleration_time = math.sqrt(self.total_distance / self.max_acceleration)
            self.max_speed = self.acceleration_time * self.max_acceleration

        self.acceleration_distance = self.max_speed * self.acceleration_time / 2.0
        self.full_speed_distance = self.total_distance - 2.0 * self.acceleration_distance
        self.full_speed_time = self.full_speed_distance / self.max_speed

        self.total_time = self.acceleration_time * 2.0 + self.full_speed_time


    def distance(self, t):
        if t < self.acceleration_time:
            return (t / self.acceleration_time) ** 2.0 * self.acceleration_distance
        elif t < self.acceleration_time + self.full_speed_time:
            return (t - self.acceleration_time) * self.max_speed + self.acceleration_distance
        elif t < self.total_time:
            return self.total_distance - ((self.total_time - t) / self.acceleration_time) ** 2.0 * self.acceleration_distance
        else:
            return self.total_distance

    def velocity(self, t):
        if t < self.acceleration_time:
            return (t / self.acceleration_time) * self.max_speed
        elif t < self.acceleration_time + self.full_speed_time:
            return self.max_speed
        elif t < self.total_time:
            return ((self.total_time - t) / self.acceleration_time) * self.max_speed
        else:
            return 0
