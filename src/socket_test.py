import socket

TCP_IP = '192.168.1.104' # this IP of my pc. When I want raspberry pi 2`s as a server, I replace it with its IP '169.254.54.195'
TCP_PORT = 4001
BUFFER_SIZE = 7 # Normally 1024, but I want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

conn, addr = s.accept()
print ('Connection address:', addr)
while 1:
    data = conn.recv(BUFFER_SIZE)
    if not data: break
    print (data.decode("utf-8"))
conn.close()