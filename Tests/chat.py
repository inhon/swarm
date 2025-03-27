import socket
import threading

# Server Code
def handle_client(client_socket, client_address):
    """Handles communication with a single client."""
    try:
        while True:
            data = client_socket.recv(1024)  # Receive data from client
            if not data:
                break  # Client disconnected
            message = data.decode("utf-8")
            print(f"Received from {client_address}: {message}")

            # Echo the message back to the client (or perform other logic)
            response = f"Server received: {message}".encode("utf-8")
            client_socket.send(response)

    except ConnectionResetError:
        print(f"Client {client_address} forcibly closed the connection.")
    except Exception as e:
        print(f"Error handling client {client_address}: {e}")
    finally:
        client_socket.close()
        print(f"Connection with {client_address} closed.")

def start_server(host, port):
    """Starts the TCP server."""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        server_socket.bind((host, port))
        server_socket.listen(5)  # Listen for up to 5 connections
        print(f"Server listening on {host}:{port}")

        while True:
            client_socket, client_address = server_socket.accept()
            print(f"Accepted connection from {client_address}")
            client_thread = threading.Thread(target=handle_client, args=(client_socket, client_address))
            client_thread.start()

    except Exception as e:
        print(f"Server error: {e}")
    finally:
        server_socket.close()

# Client Code
def start_client(host, port):
    """Starts the TCP client."""
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client_socket.connect((host, port))
        print(f"Connected to {host}:{port}")

        while True:
            message = input("Enter message (or 'exit' to quit): ")
            if message.lower() == "exit":
                break

            client_socket.send(message.encode("utf-8"))
            data = client_socket.recv(1024)
            if not data:
                print("Server disconnected.")
                break

            print(f"Received from server: {data.decode('utf-8')}")

    except ConnectionRefusedError:
        print("Connection refused. Make sure the server is running.")
    except Exception as e:
        print(f"Client error: {e}")
    finally:
        client_socket.close()

if __name__ == "__main__":
    host = "127.0.0.1"  # Localhost
    port = 12345

    server_thread = threading.Thread(target=start_server, args=(host, port))
    server_thread.daemon = True # allow the server to be killed by ctrl+c
    server_thread.start()

    start_client(host, port)