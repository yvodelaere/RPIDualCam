import io
import socket
import struct
import cv2
import numpy as np
import queue
import threading
import sys

import time

import logging
import paramiko
import os
import time





#%%Start the python script on the client using SSH


class RPIDualCam:
    def __init__(self, mode):
        hosts = ['picam1', 'picam2']
        framePorts = [9600, 9700]
        msgPorts = [9601, 9701]
        
        
        self.sshs = []
        for host in hosts:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            k = paramiko.RSAKey.from_private_key_file(host+'key')
            username = 'pi'
            ssh.connect(host, username=username, pkey = k) 
            
            if mode == 'calibrate':
                ssh.exec_command('python3 Python/rpiSide_calibrate.py')
            elif mode == 'background':
                ssh.exec_command('python3 Python/rpiSide_background.py')
            elif mode == 'operation': 
                ssh.exec_command('python3 Python/rpiSide_operation.py')
            
            self.sshs.append(ssh)
         
        self.connections = []
        self.server_sockets = []
        for framePort in framePorts:
            server_socket = socket.socket()
            server_socket.bind(('0.0.0.0', framePort))
            server_socket.listen(0)
            self.connections.append(server_socket.accept()[0].makefile('rb'))
            self.server_sockets.append(server_socket)
        
        
        self.msgSockets = []
        for msgPort in msgPorts:
            msgSocket = socket.socket()
            msgSocket.bind(('0.0.0.0', msgPort))
            msgSocket.listen(0)
            self.msgSockets.append(msgSocket)
        
        print("Connected to both cameras")
        self.last_update = time.time()
        self.keepReceiving = False



        self.queues = [queue.Queue(), queue.Queue()]
    
    def toggleCapture(self): #Send start command to RPI
    
        print("Sending start command ... ")
        for msgSocket in self.msgSockets:
            clientsocket, address = msgSocket.accept()
            string = str.encode('s')
            clientsocket.send(string)
        print("Sending command is send!")
        self.keepReceiving = not self.keepReceiving
            
        
        if self.keepReceiving == True:
            self.readThread = threading.Thread(target=self.readFramesToQueue)
            self.readThread.daemon = True
            self.readThread.start()
            
        
    def readFramesToQueue(self):
        while self.keepReceiving:
            for i, connection in enumerate(self.connections):
                _, image = self.readFrame(connection)
                self.putQueue(self.queues[i], image)
                
                
            
    def shutDown(self):
        for ssh in self.sshs:
            ssh.exec_command('pkill -9 python')
            ssh.close()
        for msgSocket in self.msgSockets:
            msgSocket.close()
        for connection in self.connections:
            connection.close()
        for server_socket in self.server_sockets:
            server_socket.close()
            
            
    def raw_resolution(self, resolution, splitter=False):
        """
        Round a (width, height) tuple up to the nearest multiple of 32 horizontally
        and 16 vertically (as this is what the Pi's camera module does for
        unencoded output).
        """
        width, height = resolution
        if splitter:
            fwidth = (width + 15) & ~15
        else:
            fwidth = (width + 31) & ~31
        fheight = (height + 15) & ~15
        return fwidth, fheight
    
    
    def bytes_to_rgb(self, data, resolution):
        yuv = self.bytes_to_yuv(data, resolution)
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB)
    
    def bytes_to_yuv(self, data, resolution):
        """
        Converts a bytes object containing YUV data to a `numpy`_ array.
        """
        width, height = resolution
        fwidth, fheight = self.raw_resolution(resolution)
        y_len = fwidth * fheight
        uv_len = (fwidth // 2) * (fheight // 2)
        # Separate out the Y, U, and V values from the array
        a = np.frombuffer(data, dtype=np.uint8)
        Y = a[:y_len].reshape((fheight, fwidth))
        Uq = a[y_len:-uv_len].reshape((fheight // 2, fwidth // 2))
        Vq = a[-uv_len:].reshape((fheight // 2, fwidth // 2))
        # Reshape the values into two dimensions, and double the size of the
        # U and V values (which only have quarter resolution in YUV4:2:0)
        U = np.empty_like(Y)
        V = np.empty_like(Y)
        U[0::2, 0::2] = Uq
        U[0::2, 1::2] = Uq
        U[1::2, 0::2] = Uq
        U[1::2, 1::2] = Uq
        V[0::2, 0::2] = Vq
        V[0::2, 1::2] = Vq
        V[1::2, 0::2] = Vq
        V[1::2, 1::2] = Vq
        # Stack the channels together and crop to the actual resolution
        return np.dstack((Y, U, V))[:height, :width] 
    
    
    def readFrame(self, connection): #Read frame from network and convert to RGB
        #Check if there is something to read
        image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
        if not image_len:
            return False, 0 
        image_stream = io.BytesIO()
        image_stream.write(connection.read(image_len))
        RGB = self.bytes_to_rgb(image_stream.getbuffer(), (2028,1520))
        return True, RGB
    
    def putQueue(self,q,frame):
        """
        Put frame in a queue object, but only store the most recent one
        """
        if not q.empty():
            try:
                q.get_nowait()
            except queue.Empty:
                pass
        q.put(frame)

    