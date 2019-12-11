#!/usr/bin/env python

import rospy
import time
import struct
from pyModbusTCP.client import ModbusClient

if __name__ == "__main__":
    c = ModbusClient(host="192.168.1.1", auto_open=True, auto_close=False, port=502, debug=False, unit_id=2)
    rospy.init_node("joint_pos_pub")

    while not rospy.is_shutdown():
        if not c.is_open():
            if not c.open():
                print("Unable to connect/nTrying to connect...")

        if c.is_open():

            j1 = c.read_holding_registers(0x0098, 2)
            j1_puu = struct.unpack('i', struct.pack('HH', j1[0], j1[1]))[0]

            j2 = c.read_holding_registers(0x009A, 2)
            j2_puu = struct.unpack('i', struct.pack('HH', j2[0], j2[1]))[0]

            j3 = c.read_holding_registers(0x009C, 2)
            j3_puu = struct.unpack('i', struct.pack('HH', j3[0], j3[1]))[0]

            j4 = c.read_holding_registers(0x009E, 2)
            j4_puu = struct.unpack('i', struct.pack('HH', j4[0], j4[1]))[0]

            j5 = c.read_holding_registers(0x0080, 2)
            j5_puu = struct.unpack('i', struct.pack('HH', j5[0], j5[1]))[0]

            j6 = c.read_holding_registers(0x0082, 2)
            j6_puu = struct.unpack('i', struct.pack('HH', j6[0], j6[1]))[0]
            print(j1_puu, j2_puu, j3_puu, j4_puu, j5_puu, j6_puu)
    c.close()