#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 18 23:16:39 2016

@author: Shuhui He
"""
import roslib  #Basic library for ROS
roslib.load_manifest('cyton_arm_controller')
from std_msgs.msg import Float64
import rospy #ROS library for python
import pygame
import time
from Tkinter import *
import socket
import serial

rospy.init_node('cyton_veta', anonymous=True)

# Global Variable
host = '192.168.32.167'
port = 5000
mySocket = socket.socket()
mySocket.bind((host,port))
mySocket.listen(1)
conn, addr = mySocket.accept()

ser2 = serial.Serial('/dev/ttyACM0', 9600)
my_position = [0,0,0,0,0,0,0,-0.95]
my_gamepad_data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
root = Tk()
root.wm_title("Robot Arm Status")
position_ctrl = [StringVar(),StringVar(),StringVar(),StringVar(),StringVar(),StringVar(),StringVar(),StringVar()]
force_ctrl = StringVar()
gripper_force = StringVar()
joint_names = ( 'shoulder_roll_controller', 'shoulder_pitch_controller', 'elbow_roll_controller',
		'elbow_pitch_controller', 'wrist_roll_controller', 'wrist_pitch_controller',
		'wrist_yaw_controller', 'gripper_open_controller')

def move(position):
	global joint_names
	#core function in moving arm
	joint_commands = tuple(position)	
	pubs = [rospy.Publisher(name + '/command', Float64) for name in joint_names]
	for i in range(len(pubs)):
		pubs[i].publish(joint_commands[i])

# 1. maintain the thread that can take out joystick data to global variable every 100ms.
def send_joypad_update_to_global():
	global my_gamepad_data, mySocket, conn, addr
	my_data_temp = conn.recv(4096)
	if not my_data_temp:
		print 'Lost Data'
	# Exclude multiple array in a received data [0,0,0][0,0,0]
	elif "][" not in my_data_temp:
		print "received data: " + my_data_temp + " **"
		my_gamepad_data = eval(my_data_temp)
	root.after(1, send_joypad_update_to_global)

# 2. maintain the thread that can get the CYTON_VETA listen to the gamepad
def cyton_veta_to_game_pad():
	global my_gamepad_data, my_position, my_previous_position
        my_previous_position = [0,0,0,0,0,0,0,-0.95]
        if -2.7 < my_position[0] + 0.05 * (my_gamepad_data[7] - my_gamepad_data[5]) < 2.7:
		my_position[0] += 0.05 * (my_gamepad_data[7] - my_gamepad_data[5])
        if -1.5 < my_position[1] + 0.05 * (my_gamepad_data[6] - my_gamepad_data[4]) < 1.5:
                my_position[1] += 0.05 * (my_gamepad_data[6] - my_gamepad_data[4])
        if -2.6 < my_position[2] + 0.05 * my_gamepad_data[2] < 2.6:
		my_position[2] += 0.05 * my_gamepad_data[2]
        if -2 < my_position[3] + -0.05 * my_gamepad_data[3] < 1.5:
		my_position[3] += -0.05 * my_gamepad_data[3]
        if -2.6 < my_position[4] + 0.05 * my_gamepad_data[0] < 2.6:
		my_position[4] += 0.05 * my_gamepad_data[0]
        if -2 < my_position[5] + 0.05 * my_gamepad_data[1] < 1.8:
		my_position[5] += 0.05 * my_gamepad_data[1]
        if -1.7 < my_position[6] + 0.05 * (my_gamepad_data[11] - my_gamepad_data[10]) < 1.8:
		my_position[6] += 0.05 * (my_gamepad_data[11] - my_gamepad_data[10])
        if -1 < my_position[7]:
		my_position[7] = -0.95 + 0.2375 * my_gamepad_data[-1]
		gripper_force_value = ser2.readline()
	if my_gamepad_data[13]==1:
		my_position = [0,0,0,0,0,0,0,-0.95]
		move(my_position)
        if my_previous_position != my_position:
		move(my_position)
                my_previous_position = my_position
	root.after(25, cyton_veta_to_game_pad)

def interface_setup():
	global position_ctrl
	w1 = Label(root, textvariable = position_ctrl[0])
	w2 = Label(root, textvariable = position_ctrl[1])
	w3 = Label(root, textvariable = position_ctrl[2])
	w4 = Label(root, textvariable = position_ctrl[3])
	w5 = Label(root, textvariable = position_ctrl[4])
	w6 = Label(root, textvariable = position_ctrl[5])
	w7 = Label(root, textvariable = position_ctrl[6])
	w8 = Label(root, textvariable = position_ctrl[7])
	w = Label(root, text = "--------------------------------------------------------------------------------------------------------------")
	w9 = Label(root, textvariable = force_ctrl)
	w10 = Label(root, textvariable = gripper_force)
	w1.pack()
	w2.pack()
	w3.pack()
	w4.pack()
	w5.pack()
	w6.pack()
	w7.pack()
	w8.pack()
	w.pack()
	w9.pack()
	w10.pack()
	root.update_idletasks()
	root.update()
	root.after(1, userinterface)

def userinterface():
	global my_position,position_ctrl, joint_names, my_gamepad_data
	i =0
	for controller in position_ctrl:
		controller.set(joint_names[i] +" position status" + " = " + str(my_position[i]))
		i+=1
	force_ctrl.set("Force applied on controller: " + str(my_gamepad_data[-1]))
	gripper_force.set("Force applied on gripper: " + ser2.readline())
	root.update_idletasks()
	root.update()
	root.after(1, userinterface)

root.after(1, interface_setup)
root.after(1, send_joypad_update_to_global)
root.after(25, cyton_veta_to_game_pad)
root.mainloop()
conn.close()

