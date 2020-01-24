#!/usr/bin/env python

import rospy
import time
import struct
import yaml
import getpass
from math import radians, pi
from sensor_msgs.msg import JointState
from pyModbusTCP.client import ModbusClient

class JointStatePublisher():
    def __init__(self):
            self.c = ModbusClient(host="192.168.1.1", auto_open=True, auto_close=False, port=502, debug=False, unit_id=2)

            rospy.init_node("joint_pos_pub")
            self.p = rospy.Publisher('/joint_states', JointState, queue_size=1)
            
            name = getpass.getuser()
            f = open('/home/%s/catkin_ws/src/delta/arm_driver/yaml/joint_limits.yaml' % name, 'r')
            d = yaml.load(f)
            f.close()

            self.dP = d["PUU_limits"]
            self.dA = d["Angle_limits"]

            self.joint_states = JointState()
            self.joint_states.name = ['shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3', 'wrist4']

    def pubAngles(self):
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")

        if self.c.is_open():
            j1 = self.c.read_holding_registers(0x0168, 2)
            j1_a = struct.unpack('i', struct.pack('HH', j1[0], j1[1]))[0]
            self.j1_angle = radians(j1_a/1000)

            j2 = self.c.read_holding_registers(0x016A, 2)
            j2_a = struct.unpack('i', struct.pack('HH', j2[0], j2[1]))[0]
            self.j2_angle = radians(j2_a/1000)

            j3 = self.c.read_holding_registers(0x016C, 2)
            j3_a = struct.unpack('i', struct.pack('HH', j3[0], j3[1]))[0]
            self.j3_angle = radians(j3_a/1000)

            j4 = self.c.read_holding_registers(0x016E, 2)
            j4_a = struct.unpack('i', struct.pack('HH', j4[0], j4[1]))[0]
            self.j4_angle = radians(j4_a/1000)

            j5 = self.c.read_holding_registers(0x0150, 2)
            j5_a = struct.unpack('i', struct.pack('HH', j5[0], j5[1]))[0]
            self.j5_angle = radians(j5_a/1000)

            j6 = self.c.read_holding_registers(0x0152, 2)
            j6_a = struct.unpack('i', struct.pack('HH', j6[0], j6[1]))[0]
            self.j6_angle = radians(j6_a/1000)

            self.joint_states.position = [self.j1_angle, self.j2_angle, -self.j3_angle, -self.j4_angle, -self.j5_angle, -self.j6_angle]
            self.joint_states.header.stamp = rospy.Time.now()
            self.p.publish(self.joint_states)
            #print(self.joint_states)

def scale(x, in_dim, out_dim):
    return (x - in_dim[0]) * (out_dim[1] - out_dim[0]) / (in_dim[1] - in_dim[0]) + out_dim[0]

if __name__ == "__main__":
    j = JointStatePublisher()
    while not rospy.is_shutdown():
        j.pubAngles()
