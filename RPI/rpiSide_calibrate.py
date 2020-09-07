import io
import picamera
import logging
import socketserver
from threading import Condition
from http import server
import threading
import time
import queue
import sys
import socket
import struct
import os




PAGE="""\
<html>
<head>
<title>picamera MJPEG streaming demo</title>
</head>
<body>
<h1>PiCamera MJPEG Streaming Demo</h1>
<img src="stream.mjpg" width="2028" height="1520" />
</body>
</html>
"""


#Add received messages to queue for processing
def add_input(input_queue, msgPort, ip):
    while True:
        s = socket.socket()
        s.connect((ip, msgPort))
        msg = s.recv(1024)
        print(msg)
        input_queue.put(msg.decode())
        s.close()

class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True





#Desktop IP
ip = "169.254.213.233"
framePort = 9700
msgPort = 9701


print("Connecting to desktop")
print(framePort)
print(msgPort)
client_socket = socket.socket()
client_socket.connect((ip, framePort))

print("Desktop is listening")
# Make a file-like object out of the connection
connection = client_socket.makefile('wb')
stream = io.BytesIO()
videoRes = (2028,1520)

keepSending = False


with picamera.PiCamera(sensor_mode=2, resolution=videoRes, framerate=24) as camera:
    imnum = 0
    output = StreamingOutput()
    camera.start_recording(output, format='mjpeg', resize=None)
    try:
        #Set up background threads
        address = ('', 8000)
        server = StreamingServer(address, StreamingHandler)
        serverThread = threading.Thread(target=server.serve_forever)
        serverThread.daemon = True
        serverThread.start()
        
        
        input_queue = queue.Queue()
        input_thread = threading.Thread(target=add_input, args=(input_queue,msgPort,ip,))
        input_thread.daemon = True
        input_thread.start()
        last_update = time.time()
        while True:
            if not input_queue.empty():
                c = input_queue.get()
                print(c)
                if c == 'x':
                    #Send capture to client
                    print("Capture now")
                    camera.capture(stream, 'yuv')
                    #stream.tell() returns the number of bytes
                    connection.write(struct.pack('<L', stream.tell()))
                    connection.flush()
                    #Reset stream
                    stream.seek(0)
                    connection.write(stream.read())
                    #Prepare stream for next send
                    stream.seek(0)
                    stream.truncate()
                    
                elif c == 's':
                    keepSending = not keepSending
                    print("KeepSending is " + str(keepSending))
                    
                    
                else:
                    print("Received message: " + c + " But no action is taken")
            if keepSending:
                #Send capture to client
                print("Capture now")
                camera.capture(stream, 'yuv')
                #stream.tell() returns the number of bytes
                connection.write(struct.pack('<L', stream.tell()))
                connection.flush()
                #Reset stream
                stream.seek(0)
                connection.write(stream.read())
                #Prepare stream for next send
                stream.seek(0)
                stream.truncate()
            
    finally:
        print("stopping")
        camera.stop_recording()