#!/usr/bin/env python

import rospy
import yaml
import getpass
from geometry_msgs.msg import Wrench, WrenchStamped
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

class FT_Sensor():
    """This is the FT_Sensor class. This class is used for the FT300 sensor with ROS.
    """
    def __init__(self):
        self.sensor = ModbusClient(method='rtu', port='/dev/ttyUSB0', baudrate=19200, timeout=1, stopbits=1)
        self.sensor.connect()
        self.name = getpass.getuser()
        f = open('/home/%s/catkin_ws/src/delta/ft300_sensor/yaml/calibration.yaml' % self.name, 'r')
        self.d = yaml.load(f)
        f.close()
        self.pub = rospy.Publisher('/ft_data', Wrench, queue_size=10)
        self.stamped_pub = rospy.Publisher('/stamped_ft_data', WrenchStamped, queue_size=10)
        rospy.init_node("forcetorque_publisher")
        self.data = Wrench()
        self.stamped_data = WrenchStamped()

    def pubVals(self):
        """This function publishes the force and torque values measured by the forcetorque sensor to the /ft_data and /stamped_ft_data topics.
        The published messages are Wrench and WrenchStamped.
        """
        l = []
        for registers in self.sensor.read_holding_registers(180, 6, unit = 0x0009).registers:
            l.append(twos_comp(registers, 16))
        self.data.force.x = l[0] - self.d['fx']
        self.data.force.y = l[1] - self.d['fy']
        self.data.force.z = l[2] - self.d['fz']
        self.data.torque.x = l[3]- self.d['mx']
        self.data.torque.y = l[4] - self.d['my']
        self.data.torque.z = l[5] - self.d['mz']
        self.stamped_data.wrench = self.data
        self.stamped_data.header.frame_id = "EE/ft300"
        self.stamped_data.header.stamp = rospy.Time.now()
        self.pub.publish(self.data)
        self.stamped_pub.publish(self.stamped_data)

    def calibrate(self):
        """This function is used to set the 0 values of the sensor. When this function is called the current force torque values will be set as 0.
        """
        l = []
        for registers in self.sensor.read_holding_registers(180, 6, unit = 0x0009).registers:
            l.append(twos_comp(registers, 16))
        di = {'fx':l[0], 'fy':l[1], 'fz':l[2], 'mx':l[3], 'my':l[4], 'mz':l[5]}
        f = open('/home/%s/catkin_ws/src/delta/ft300_sensor/yaml/calibration.yaml' % self.name, 'w')
        yaml.dump(di, f, default_flow_style=False)
        f.close()
        f = open('/home/%s/catkin_ws/src/delta/ft300_sensor/yaml/calibration.yaml' % self.name, 'r')
        self.d = yaml.load(f)
        f.close()

def twos_comp(val, bits):
    """This function is used to calculate the twos complement of a value.
    
    Arguments:
        val {int} -- input value to calculate the twos complement from.
        bits {int} -- The amount of bits for the twos complement calculation.
    
    Returns:
        [int] -- The twos complement of the input value.
    """
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val

if __name__ == "__main__":
    s = FT_Sensor()
    s.calibrate()
    while not rospy.is_shutdown():
        s.pubVals()