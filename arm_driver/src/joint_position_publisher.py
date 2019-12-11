#!/usr/bin/env python

import rospy
import time
from pyModbusTCP.client import ModbusClient

if __name__ == "__main__":
    c = ModbusClient(host="192.168.1.1", auto_open=True, auto_close=False, port=502, debug=True, unit_id=2)
    c.mode(2)
    rospy.init_node("joint_pos_pub", anonymous=True)

    while not rospy.is_shutdown():
        if not c.is_open():
            if not c.open():
                print("Unable to connect/nTrying to connect...")

        if c.is_open():
            print("Connected")
            bits = c.read_coils(152, 16) #Read double word (32 bits)
            print(bits)

        time.sleep(2)
    c.close()