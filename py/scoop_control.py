import serial
import time
from SerialPacket import *
from saleae_capture import SaleaeCaptureSession
import os
import re


#Read Waypoints
def read_waypoint(filename):
    coordinates = []
    with open(filename, 'r') as file:
        for line in file:
            x,y,pitch,vibe = map(float, line.strip().split(','))
            coordinates.append((x,y,pitch,vibe))
    return coordinates 

def get_next_filename(directory, base_filename="test_", extension=".csv"):
    files = [f for f in os.listdir(directory) if f.startswith(base_filename) and f.endswith(extension)]
    
    numbers = []
    for filename in files:
        match = re.match(rf"{base_filename}(\d+){extension}", filename)
        if match:
            numbers.append(int(match.group(1)))

    if not numbers:
        return os.path.join(directory, f"{base_filename}1{extension}")
    
    next_number = max(numbers) + 1
    return os.path.join(directory, f"{base_filename}{next_number}{extension}")

class Scoop:
    
    def __init__(self):
        self.ser = serial.Serial(port = "COM12", baudrate = 115200)
        self.command = Command.COMMAND_STOP
        self.x = 0.0
        self.y = 0.0
        self.pitch = 0.0
        self.vibe = 0.0
        self.record_data = True 
        self.capture_session = None  # Instance of SaleaeCaptureSession


    def update_values(self, pitch = 0.0, vibe = 0.0):
        self.pitch = pitch
        self.vibe = vibe

    def idle(self):
        self.command = Command.COMMAND_IDLE
        send_packet(self.ser, self.command, self.x, self.y, self.pitch, self.vibe)
    
    def start(self):
        self.command = Command.COMMAND_START
        send_packet(self.ser, self.command, self.x, self.y, self.pitch, self.vibe)
        
    def stop(self):
        self.command = Command.COMMAND_STOP
        send_packet(self.ser, self.command, self.x, self.y, self.pitch, self.vibe)


    def home(self):
        self.command = Command.COMMAND_HOME
        send_packet(self.ser, self.command, self.x, self.y, self.pitch, self.vibe)

    def move_position(self, x, y, pitch, vibe):
        self.x = x
        self.y = y
        self.pitch = pitch
        self.vibe = vibe

    def dig_sequence(self):
        self.command = Command.COMMAND_MOVE_WAYPOINT
        traj = read_waypoint('waypoints.txt')
        i = 0

        if self.record_data:
            self.capture_session = SaleaeCaptureSession()
            self.capture_session.start_capture()

        try:
            while i < len(traj):

                x, y, pitch, vibe = traj[i]
                send_packet(self.ser, command=self.command, x=x, y=y, pitch=pitch, vibe=vibe)
                time.sleep(0.05)
                i += 1

            time.sleep(1)
            self.idle()

            if self.record_data:
                self.capture_session.end_capture()

        finally:
            self.ser.close()

     
            


    def manual_control(self):
        self.command = Command.COMMAND_MOVE_MANUAL
        send_packet(self.ser, self.command, self.x, self.y, self.pitch, self.vibe)


        

