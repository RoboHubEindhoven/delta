#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Wrench
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

class FT_Sensor():
    def __init__(self):
        self.pub = rospy.Publisher('/ft_data', Wrench, queue_size=10)
        rospy.init_node("forcetorque_publisher")
        self.data = Wrench()
        self.sensor = ModbusClient(method='rtu', port='/dev/ttyUSB0', baudrate=19200, timeout=1, stopbits=1)
        self.sensor.connect()

    def pubVals(self):
        l = []
        for registers in self.sensor.read_holding_registers(180, 6, unit = 0x0009).registers:
            l.append(twos_comp(registers, 16))
        self.data.force.x = l[0]
        self.data.force.y = l[1]
        self.data.force.z = l[2]
        self.data.torque.x = l[3]
        self.data.torque.y = l[4]
        self.data.torque.z = l[5]
        self.pub.publish(self.data)

def twos_comp(val, bits):
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val

if __name__ == "__main__":
    s = FT_Sensor()
    while not rospy.is_shutdown():
        s.pubVals()