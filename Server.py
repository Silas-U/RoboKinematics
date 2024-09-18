import socket
import math
import json

# Configure the ESP32 as an access point or connect to a Wi-Fi network.
ssid = 'silasudofia469@gmail.com'
password = 'web2web3'

import network

sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect(ssid, password)

while not sta_if.isconnected():
    pass

print('Network Config:', sta_if.ifconfig())

# Set up the socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('', 12345))  # Bind to the IP and port
server_socket.listen(1)  # Listen for incoming connections

print("Waiting for a connection...")

# Accept a connection
client_socket, client_address = server_socket.accept()
print(f"Connection from {client_address}")

dat=[]

try:
    while True:
        data = client_socket.recv(4096)  # Buffer size is 1024 bytes
        if not data:
            print("Connection closed by client")
            break  # Exit the loop if the client disconnects
        
        str_data = data.decode('utf-8')
        
        # Deserialize the JSON-encoded message
        received_list = json.loads(str_data)

        dat.append(received_list)
        
        print(dat)
        

        # Echo back the received data
        client_socket.send(data)

finally:
    
    client_socket.close()
    server_socket.close()
    
    T1 = [i*(180/math.pi) for i in dat[0]]
    T2 = [i*(180/math.pi) for i in dat[1]]
    
    for i in range(len(T1)):
        print(T1[i],T2[i])
