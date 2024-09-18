import socket

# Replace with the IP address of your ESP32
esp32_ip = '192.168.43.5'
port = 12345

# Set up the socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((esp32_ip, port))

try:
    # Send a message
    message = "Hello Silas!"
    client_socket.sendall(message.encode())

    # Receive the response
    data = client_socket.recv(1024)
    print(f"Received from ESP32: {data.decode()}")
finally:
    client_socket.close()

