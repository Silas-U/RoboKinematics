import socket
import time

# Replace with the IP address of your ESP32
esp32_ip = '192.168.43.5'
port = 12345

# Set up the socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((esp32_ip, port))

i=0
try:
    while True:  # Send 10 messages as an example
        i=i+1
        message = f"Message {i + 1}"
        client_socket.sendall(message.encode())
        
        # Receive the response (optional, if the server echoes back)
        data = client_socket.recv(1024)
        print(f"Received from ESP32: {data.decode()}")
        
        time.sleep(1)  # Wait for 1 second before sending the next message

finally:
    client_socket.close()
