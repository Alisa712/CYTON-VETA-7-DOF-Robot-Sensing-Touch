import socket
import time

import pygame

pygame.init()
joy = pygame.joystick.Joystick(0)
joy.init()

mySocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def joystick():
	global out
	out = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	it = 0 #iterator
	pygame.event.pump()

	#Read input from the two joysticks	   
	for i in range(0, joy.get_numaxes()):
		out[it] = joy.get_axis(i)
		it+=1
	#Read input from buttons
	for i in range(0, joy.get_numbuttons()):
		out[it] = joy.get_button(i)
		it+=1
	return out

while True:
	message = joystick()
	mySocket.sendto(str(message), ('localhost', 5000))
	print message
	time.sleep(0.05)

