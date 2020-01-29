#!/usr/bin/env python
#
# https://www.dexterindustries.com/BrickPi/
# https://github.com/DexterInd/BrickPi3
#
# Copyright (c) 2016 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information, see https://github.com/DexterInd/BrickPi3/blob/master/LICENSE.md
#
# This code is an example for running a motor to a target position set by the encoder of another motor.
# 
# Hardware: Connect EV3 or NXT motors to the BrickPi3 motor ports B and C. Make sure that the BrickPi3 is running on a 9v power supply.
#
# Results:  When you run this program, motor C power will be controlled by the position of motor B. Manually rotate motor B, and motor C's power will change.

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import math

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

def light():
    try:
        value = BP.get_sensor(BP.PORT_1)
        return value
    except brickpi3.SensorError as error:
        print(error)
        return -1

def on_path():
    value = light()
    if value > 2200:
        return 2
    elif value <= 2200 and value >= 2100: 
        return 1
    else:
        return 0
def reset_encoders():
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))

def encoder1():
    return BP.get_motor_encoder(BP.PORT_C)

def encoder2():
    return BP.get_motor_encoder(BP.PORT_B)

def drive(motor1, motor2):
    BP.set_motor_power(BP.PORT_C, motor1-2)
    BP.set_motor_power(BP.PORT_B, motor2)

def drive_odom(motor1, motor2):
    BP.set_motor_power(BP.PORT_C, 1.5*(motor1-1))
    BP.set_motor_power(BP.PORT_B, 1.5*motor2)

def test():
    try:
        try:
            BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # reset encoder B
        except IOError as error:
            print(error)
    
        while True:
            # The following BP.get_motor_encoder function returns the encoder value (what we want to use to control motor C's power).
            try:
                power = BP.get_motor_encoder(BP.PORT_B) / 10
                if power > 100:
                    power = 100
                elif power < -100:
                    power = -100
            except IOError as error:
                print(error)
                power = 0
            BP.set_motor_power(BP.PORT_C, power)
        
            time.sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.

    except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
        BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.

def follow_light():
    BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_LIGHT_ON)
    reset_encoders()
    count = 0
    try:
        while True:
            path = on_path()
            if path == 2:
                count += 1
                drive(-30, 30)
            elif path == 1:
                drive(30, 30)
            else:
                if count > 10:
                    print(count)
                count = 0
                drive(30, -30)
            time.sleep(0.02)
    except KeyboardInterrupt:
        BP.reset_all()

def odometry(moves):
    reset_encoders()
    tstep = 0.05
    tcount = 0
    movecount = 0
    radius = .85
    width = 6.1
    prev_click_left = 0
    prev_click_right = 0
    prev_theta = 0
    prev_x = 0
    prev_y = 0
    motorLeft = moves[movecount][0]
    motorRight = moves[movecount][1]
    try:
        while True:
            drive_odom(motorLeft, motorRight)
            click_left = encoder1()
            click_right = encoder2()
            angular_left = (click_left - prev_click_left) / tstep
            angular_right = (click_right - prev_click_right) / tstep
            angular_left = angular_left * math.pi/180
            angular_right = angular_right * math.pi/180
        
            v = radius * (angular_left + angular_right) / 2
            angular = radius * (angular_right - angular_left) / width
        
            k00 = v * math.cos(prev_theta)
            k01 = v * math.sin(prev_theta)
            k02 = angular

            k10 = v * math.cos(prev_theta + (tstep/2) * k02)
            k11 = v * math.sin(prev_theta + (tstep/2) * k02)
            k12 = angular

            k20 = v * math.cos(prev_theta + (tstep/2) * k12)
            k21 = v * math.sin(prev_theta + (tstep/2) * k12)
            k22 = angular

            k30 = v * math.cos(prev_theta + tstep * k22)
            k31 = v * math.sin(prev_theta + tstep * k22)
            k32 = angular

            x = prev_x + (tstep/6) * (k00 + 2 * (k10 + k20) + k30)
            y = prev_y + (tstep/6) * (k01 + 2 * (k11 + k21) + k31)
            theta = prev_theta + (tstep/6) * (k02 + 2 * (k12 + k22) + k32)
            prev_x = x
            prev_y = y
            prev_theta = theta
            prev_click_left = click_left
            prev_click_right = click_right
            time.sleep(tstep)
            tcount+=1
            if tstep*tcount >= 3:
                tcount = 0
                movecount += 1
                print(x)
                print(y)
                print(theta)
                drive(2, 0)
                time.sleep(1)
                if movecount >= len(moves):
                    print("Final X: " + str(x))
                    print("Final Y: " + str(y))
                    print("Final Theta: " + str(theta*180/math.pi))
                    BP.reset_all()
                    return
                motorLeft = moves[movecount][0]
                motorRight = moves[movecount][1]

    except KeyboardInterrupt:
        BP.reset_all()

def main():
    pass
    #follow_light()
    #odometry([(40, 40), (-20, 30), (30, 30)])
#main()
