import socket
import time
import pygame
import serial 

pygame.init()
joy = pygame.joystick.Joystick(0)
joy.init()

ser = serial.Serial('/dev/ttyACM0', 9600)

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
	try:			
		pressure = float(ser.readline())
		# Add pressure to the list		
		message.append(pressure)
		mySocket.sendto(str(message), ('192.168.32.167', 5000))
		print str(message)		
	except ValueError, e:		
		pressure = 0.0 		
		message.append(pressure)
		mySocket.sendto(str(message), ('192.168.32.167', 5000))
	#time.sleep(0.025)
	

