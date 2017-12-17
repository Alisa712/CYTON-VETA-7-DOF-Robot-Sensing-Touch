import socket

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('localhost', 5000))

print 'listening on: %s : %d' % ('localhost', 5000)

while True:
	d = s.recvfrom(4096)
	data = d[0]
	addr = d[1]
	if not data:
		print 'Lost Data'
	data = eval(data)
	print 'From address: ' + str(addr)
	print 'gamepad input: ' + str(data)
