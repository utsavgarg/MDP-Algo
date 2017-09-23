import socket
import time
import sys

# class WFConnector:

# host = '192.168.26.1'
# port = 8008
#
# s = None
# for res in socket.getaddrinfo(host, port, socket.AF_UNSPEC, socket.SOCK_STREAM):
#     family, socket_type, proto, canonname, socket_address = res
#     try:
#         s = socket.socket(family, socket_type, proto)
#     except socket.error:
#         s = None
#         continue
#     try:
#         s.connect(socket_address)
#     except socket.error:
#         s.close()
#         s = None
#         continue
#     break
# if s is None:
#     print('could not open socket')
#
# while 1:
#
#     dataToBeSent = str.encode(input("Input string: "))
#     s.sendall(dataToBeSent)
#
#     data = s.recv(1024)
#     print('Received', repr(data))
# s.close()


class Connector:
    def __init__(self):

        family = socket.AF_INET
        socket_type = socket.SOCK_STREAM
        self.socket = socket.socket(family, socket_type)
        self.socket.settimeout(1)
        self.connected = False
        self.connect()

    def connect(self):

        # for res in socket.getaddrinfo(host, port, socket.AF_UNSPEC, socket.SOCK_STREAM):
        #     family, socket_type, proto, canonname, socket_address = res
        #     try:
        #         s = socket.socket(family, socket_type, proto)
        #     except socket.error:
        #         continue
        #     try:
        #         s.connect(socket_address)
        #     except socket.error:
        #         s.close()
        #         continue
        #     s.settimeout(0.5)
        #     # s.setblocking(False)
        #     self.socket = s
        #     print("[Info] Connection established.")
        #     print("         family: ", family)
        #     print("         socket_type: ", socket_type)
        #     print("         proto: ", proto)
        #     break
        host = '192.168.26.1'
        port = 8008
        try:
            self.socket.connect((host, port))
        except Exception:
            print("[Error] Unable to establish connection.")
        else:
            self.connected = True
            print("[Info] Connection established.")

    def send(self, msg):
        if not self.connected:
            self.connect()
        if self.connected:
            print("[Info] Sending message: ", msg)
            try:
                self.socket.sendall(str.encode(msg))
            except Exception:
                print("[Error] Unable to send message. Connection loss.")
                # self.connected = False

    def receive(self):
        if not self.connected:
            self.connect()
        if self.connected:
            try:
                msg = self.socket.recv(1024)
                if msg:
                    print("[Info] Received: ", msg.decode())
                return msg
            except socket.timeout:
                print("No message is received.")
        # else:
        #     print("[Error] Unable to receive message. Connection loss.")


connector = Connector()
while True:
    dataToBeSent = input("Input string: ")
    connector.send(dataToBeSent)
    connector.receive()