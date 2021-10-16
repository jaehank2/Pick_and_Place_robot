#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from project_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
# home = np.radians([120, -90, 90, -90, -90, 0])

home = np.radians([151.30, -107.46, 116.14, -101.53, -92.15, 35.33])

# Hanoi tower location 1
# Q11 = [120*pi/180.0, -56*pi/180.0, 124*pi/180.0, -158*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q12 = [120*pi/180.0, -64*pi/180.0, 123*pi/180.0, -148*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q13 = [120*pi/180.0, -72*pi/180.0, 120*pi/180.0, -137*pi/180.0, -90*pi/180.0, 0*pi/180.0]


# 1 - low to high
Q11 = [136.14*pi/180.0, -61.1*pi/180.0, 141.09*pi/180.0, -173.2*pi/180.0, -90.67*pi/180.0, 19.99*pi/180.0]
Q12 = [136.15*pi/180.0, -72.61*pi/180.0, 140.01*pi/180.0, -160.59*pi/180.0, -90.81*pi/180.0, 19.97*pi/180.0]
Q13 = [136.13*pi/180.0, -82.54*pi/180.0, 137.42*pi/180.0, -148.07*pi/180.0, -90.96*pi/180.0, 19.95*pi/180.0]
# Q13 = [136.12*pi/180.0, -98.74*pi/180.0, 124.83*pi/180.0, -119.29*pi/180.0, -91.22*pi/180.0, 20.04*pi/180.0]

# 2 - low to high
Q21 = [151.31*pi/180.0, -62.2*pi/180.0, 145.82*pi/180.0, -176.49*pi/180.0, -91.46*pi/180.0, 35.15*pi/180.0]
Q22 = [151.33*pi/180.0, -74.98*pi/180.0, 144.63*pi/180.0, -162.52*pi/180.0, -91.61*pi/180.0, 35.13*pi/180.0]
Q23 = [151.33*pi/180.0, -86.37*pi/180.0, 141.51*pi/180.0, -148*pi/180.0, -91.77*pi/180.0, 35.14*pi/180.0]

# 3 - low to high
Q31 = [169.86*pi/180.0, -61.61*pi/180.0, 143.19*pi/180.0, -173.76*pi/180.0, -92.3*pi/180.0, 53.69*pi/180.0]
Q32 = [169.88*pi/180.0, -73.47*pi/180.0, 142.09*pi/180.0, -160.8*pi/180.0, -92.44*pi/180.0, 53.67*pi/180.0]
Q33 = [169.88*pi/180.0, -84.11*pi/180.0, 139.19*pi/180.0, -147.26*pi/180.0, -92.59*pi/180.0, 53.69*pi/180.0]

# starting and ending index to store user input
start_idx = 0
mid_idx = 0
end_idx = 0
counter = 0
from_height = [2, 1, 0, 0, 1, 0, 0]
to_height = [0, 0, 1, 0, 0, 1, 2]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0                    # set to true if gripper is holding block
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False                # last state of the gripper - suction On or Off
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def gripper_callback(msg):
    global digital_in_0
    global analog_in_0

    digital_in_0 = msg.DIGIN





############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True

# suction on = true, suction off = false
def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q
    global home
    global digital_in_0

    error = 0

    # move arm to home first
    move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)

    # move arm to start location and turn suction on
    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height], 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_on)
    # Delay to make sure suction cup has grasped the block
    time.sleep(1.0)

    print("*****")
    print("moving from", start_loc, "height", start_height, "to", end_loc, "height", end_height, "digin", digital_in_0)
    if digital_in_0 == 0:
        # rospy.logerr("BLOCK NOT FOUND. System quitting...")
        rospy.loginfo("BLOCK NOT FOUND. System quitting...")
        gripper(pub_cmd, loop_rate, suction_off)
        sys.exit()

    # move back to home before going to end location
    move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)

    # move arm to end location and turn suction off
    move_arm(pub_cmd, loop_rate, Q[end_loc][end_height], 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(1.0)


    return error


#### custom helper function for main
def solveTowerOfHanoi(start, mid, end, height, pub_command, loop_rate, disk):
    global home
    global Q
    global SPIN_RATE
    global counter
    global from_height
    global to_height

    if disk == 1:
        start_height = from_height[counter]
        end_height = to_height[counter]
        move_block(pub_command, loop_rate, start, start_height, end, end_height)
        counter += 1
        return
    else:
        solveTowerOfHanoi(start, end, mid, height, pub_command, loop_rate, disk-1)
        start_height = from_height[counter]
        end_height = to_height[counter]
        move_block(pub_command, loop_rate, start, start_height, end, end_height)
        counter += 1
        solveTowerOfHanoi(mid, start, end, height, pub_command, loop_rate, disk-1)



#### end of custom helper function
def move_linear(pub_cmd, loop_rate, height, vel, accel):
    for i in range(3):
        move_arm(pub_cmd, loop_rate, Q[i][height], vel, accel)


def main():

    global home
    global Q
    global SPIN_RATE

    global start_idx
    global mid_idx
    global end_idx
    global counter

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)



    # TODO: modify the code below so that program can get user input

    input_done = 0
    loop_count = 0
        
        
    # get user input for number of loops
    while(not input_done):
        input_string = raw_input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string + "\n")

        if(int(input_string) == 1):
            input_done = 1
            loop_count = 1
        elif (int(input_string) == 2):
            input_done = 1
            loop_count = 2
        elif (int(input_string) == 3):
            input_done = 1
            loop_count = 3
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")



    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    

    while(loop_count > 0):
        # first move arm to home location
        move_arm(pub_command, loop_rate, home, 4.0, 4.0)

        ### test
        # move_arm(pub_cmd, loop_rate, dest, vel, accel)
        move_linear(pub_command, loop_rate, 0, 4.0, 4.0)
        
        move_arm(pub_command, loop_rate, home, 4.0, 4.0)

        move_linear(pub_command, loop_rate, 1, 4.0, 4.0)

        move_arm(pub_command, loop_rate, home, 4.0, 4.0)

        move_linear(pub_command, loop_rate, 2, 4.0, 4.0)

        move_arm(pub_command, loop_rate, home, 4.0, 4.0)

        ### end of test



        loop_count = loop_count - 1


    gripper(pub_command, loop_rate, suction_off)




if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
