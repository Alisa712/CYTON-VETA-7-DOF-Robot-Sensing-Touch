import socket
import time

import pygame

pygame.init()
joy = pygame.joystick.Joystick(0)
joy.init()

host = 'localhost'
port = 5000

mySocket = socket.socket()
mySocket.connect((host,port))

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
	mySocket.send(str(message))
	print message

	time.sleep(0.5)

mySocket.close()

