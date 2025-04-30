#!/usr/bin/env python3

import socket
import threading
import time


def server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 14550))
    print("UDP server listening on 0.0.0.0:14550")
    while True:
        data, addr = sock.recvfrom(1024)
        print(f"Received message: {data.decode()} from {addr}")


def client():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        message = f"Hello, UDP! Time: {time.time()}"
        sock.sendto(message.encode(), ('127.0.0.1', 14550))
        print(f"Sent message: {message}")
        time.sleep(1)


if __name__ == "__main__":
    server_thread = threading.Thread(target=server)
    client_thread = threading.Thread(target=client)

    server_thread.start()
    time.sleep(1)  # Give the server a moment to start
    client_thread.start()

    server_thread.join()
    client_thread.join()