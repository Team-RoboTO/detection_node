from pickletools import uint8
from serial.tools.list_ports import comports
import struct
import serial
import numpy as np

class Ser:
    def __init__(self):
        self.classes = ["red", "blue"]
        port = comports()
        name = port[0].device
        print("Connection to:", name)
        self.ser= serial.Serial(name,115200, timeout=0.001)
        print("done\n")
        
    def start_comunication_and_get_color(self):
        i = 0
        enemy_color = None
        initial_coordinates =[0,0,0]
        bts=[]
        for f in initial_coordinates:
            for byte in struct.pack('<f',float(f)):
                bts.append(byte)

        bts.append(np.uint8(1))

        b=bytearray(bts)
        self.ser.write(b)
        
        print("Waiting for signal...")

        i = self.read_A()
        print(i)
        enemy_color = self.classes[i-1]

        print("Signal received...\nOur color is:", enemy_color )
        return enemy_color

    
    def send_everything(self, s, control):
        bts=[]
        for f in s:
            for byte in struct.pack('<f',float(f)):
                bts.append(byte)
        
        bts.append(np.uint8(control))
        
        print(f"s: {s}, control: {control}")
        b=bytearray(bts)
        self.ser.write(b)
        self.ser.read(1)

    def read_A(self):
        a=self.ser.read(size=1)
        return int.from_bytes(a, byteorder='little',signed=False)

    def is_Open(self):
        return self.ser.isOpen()
