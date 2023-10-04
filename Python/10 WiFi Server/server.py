import socket, time

HOST = '192.168.127.148'
PORT = 3000
socket.setdefaulttimeout(3)
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((HOST, PORT))
s.listen(12)

client, address = s.accept()
print(address[0] + ' Connected.')

while True:
    client.send(b'?')
    time.sleep(0.1)
    data = client.recv(100)
    print(data)