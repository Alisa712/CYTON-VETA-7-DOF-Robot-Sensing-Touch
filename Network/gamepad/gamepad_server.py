import socket

host = 'localhost'
port = 5000

mySocket = socket.socket()
mySocket.bind((host,port))

mySocket.listen(1)
conn, addr = mySocket.accept()
print 'connection from: ' + str(addr)

while True:
	data = conn.recv(4096)
	if not data:
		print 'Lost Data'
	data = eval(data)
	print 'gamepad input: ' + str(data)
	print 'test: ' + str(data[0]+2)
	
conn.close()
