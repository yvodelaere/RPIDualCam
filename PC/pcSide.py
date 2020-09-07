import io
import socket
import struct
from PIL import Image
import cv2
import numpy as np
from PIL import Image
import queue
import threading
import sys

import time

import logging
import paramiko
import os
import time
import queue


def putQueue(q,frame):
    """
    Put frame in a queue object, but only store the most recent one
    """
    if not q.empty():
        try:
            q.get_nowait()
        except queue.Empty:
            pass
    q.put(frame)

      
      


def add_input(input_queue):
    while True:
        input_queue.put(input())
        
        
def raw_resolution(resolution, splitter=False):
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


def bytes_to_rgb(data, resolution):
    yuv = bytes_to_yuv(data, resolution)
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB)

def bytes_to_yuv(data, resolution):
    """
    Converts a bytes object containing YUV data to a `numpy`_ array.
    """
    width, height = resolution
    fwidth, fheight = raw_resolution(resolution)
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


def readFrame(connection): #Read frame from network and convert to RGB
    #Check if there is something to read
    image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
    if not image_len:
        return False, 0 
    image_stream = io.BytesIO()
    image_stream.write(connection.read(image_len))
    RGB = bytes_to_rgb(image_stream.getbuffer(), (2028,1520))
    return True, RGB
    


#%%Start the python script on the client using SSH
    
hosts = ['picam1', 'picam2']
sshs = []
for host in hosts:
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    k = paramiko.RSAKey.from_private_key_file(host+'key')
    username = 'pi'
    ssh.connect(host, username=username, pkey = k) 
    ssh.exec_command('python3 Python/rpiSide.py')
    sshs.append(ssh)





camNames = ['_C1', '_C2'] #Names used for saving images
 

framePorts = [9600, 9700]
msgPorts = [9601, 9701]

    
connections = []
server_sockets = []
for framePort in framePorts:
    server_socket = socket.socket()
    server_socket.bind(('0.0.0.0', framePort))
    server_socket.listen(0)
    connections.append(server_socket.accept()[0].makefile('rb'))
    server_sockets.append(server_socket)



msgSockets = []
for msgPort in msgPorts:
    msgSocket = socket.socket()
    msgSocket.bind(('0.0.0.0', msgPort))
    msgSocket.listen(0)
    msgSockets.append(msgSocket)



print("Starting")
input_queue = queue.Queue()

input_thread = threading.Thread(target=add_input, args=(input_queue,))
input_thread.daemon = True
input_thread.start()

last_update = time.time()

keepReceiving = False


queues = [queue.Queue(), queue.Queue()]





        
try:
    while True:
        if time.time()-last_update>0.5:
            print('.')
            last_update = time.time()

        if not input_queue.empty():
            c = input_queue.get()
            print("Sending message: " + c + " to servers")
            for msgSocket in msgSockets:
                clientsocket, address = msgSocket.accept()
                string = str.encode(c)
                clientsocket.send(string)
            if c == 'x':
                print("Requesting frame from camera")
                for i, connection in enumerate(connections):
                    _, image = readFrame(connection)
                    im = Image.fromarray(image)
                    im.verify()
                    print('Received image from camera ' + str(i))
            if c == 's':
                keepReceiving = not keepReceiving
                print("Keep receiving is: " + str(keepReceiving))
            
        if keepReceiving:
            for i, connection in enumerate(connections):
                _, image = readFrame(connection)
                #Show in windows image viewer
                im = Image.fromarray(image)
                #Here, the images should be placed into an queue
                putQueue(queues[i], im)
                print('Received image in continious mode from camera ' + str(i))
                
                
                
                
                
            
                    
                    
         
        
        
finally:
    for ssh in sshs:
        ssh.exec_command('pkill -9 python')
        ssh.close()
    for msgSocket in msgSockets:
        msgSocket.close()
    for connection in connections:
        connection.close()
    for server_socket in server_sockets:
        server_socket.close()