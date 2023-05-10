from pickletools import uint8
from rclpy.node import Node
from serial.tools.list_ports import comports
import struct
import serial
import numpy as np
import rclpy

from std_msgs.msg import Float32MultiArray, MultiArrayDimension

class Ser:
    def __init__(self):
        self.classes = ["red", "blue"]
        port = comports()
        name = port[0].device
        print("Connection to:", name)
        self.ser= serial.Serial(name,230400, timeout=0.0001)
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

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/yolo/enemy_pose',
            self.listener_callback,
            5
        )
        self.serial = Ser()

    def listener_callback(self, msg):
        position = np.array(msg.data, dtype=np.float32)
        print("sono nella callback")
        if np.all(position != 0):
            print("sto inviado")
            self.serial.send_everything(position[:3],1)

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()