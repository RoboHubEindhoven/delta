#!/usr/bin/env python

from pyModbusTCP.client import ModbusClient
from math import radians
import time
import numpy as np
import rospy
import struct
import yaml
import getpass

from arm_driver.srv import reset_errors, power, teach_position
from end_effector import EndEffector

class DRV90L():
    """The "DRV90L" class contains methods to control the robot.
    """
    def __init__(self):
        rospy.init_node("RobotArm")
        ip = rospy.get_param("/robot_ip")
        self.resetService = rospy.Service('/reset_robot', reset_errors, self.resetErrors)
        self.powerService = rospy.Service('/power_robot', power, self.powerCallback)
        self.teachService = rospy.Service("/teach_position", teach_position, self.teachCurrentToolPose)
        self.c = ModbusClient(host=ip, auto_open=True, auto_close=False, port=502, debug=False, unit_id=2)
        self.bits = 0
        self.name = getpass.getuser()

    def enableRobot(self):
        """This function enables the robot. After this function is called, the robot is ready to move.
        !!!Be aware the motors get enabled after calling this function!!!
        """
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")

        if self.c.is_open():
            self.c.write_single_register(0x0006, 0x101)
            self.c.write_single_register(0x0007, 0x101)
            self.c.write_single_register(0x0000, 0x101)
            print("Enabling robot...")
            time.sleep(3)

    def disableRobot(self):
        """This function disables the robot. After this function is called, the robot can't move. The motors aren't powered anymore.
        """
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")

        if self.c.is_open():
            self.c.write_single_register(0x0006, 0x000)
            self.c.write_single_register(0x0007, 0x000)
            self.c.write_single_register(0x0000, 0x000)
            time.sleep(3)
            print("Robot is disabled")

    def sendPositionMove(self, x, y, z, rx, ry, rz, speed, frame):
        """This function is used to move the end effector of the robot to a certain position.
        !!!The robot needs to be enables for this function to work!!!
        
        Arguments:
            x {int} -- [x position in mm]
            y {int} -- [y position in mm]
            z {int} -- [z position in mm]
            rx {int} -- [rotation around x axis in degrees]
            ry {int} -- [rotation around y axis in degrees]
            rz {int} -- [rotation around z axis in degrees]
            speed {int} -- [speed in % [0 to 100]]
            frame {string} -- [Reference frame (world is base of robot)]
        """
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")

        if self.c.is_open():
            xu = int(np.binary_repr(x*1000, width=32), 2)
            yu = int(np.binary_repr(y*1000, width=32), 2)
            zu = int(np.binary_repr(z*1000, width=32), 2)
            rxu = int(np.binary_repr(rx*1000, width=32), 2)
            ryu = int(np.binary_repr(ry*1000, width=32), 2)
            rzu = int(np.binary_repr(rz*1000, width=32), 2)
            xh = xu >> 16
            xl = xu & 0x0000FFFF
            yh = yu >> 16
            yl = yu & 0x0000FFFF
            zh = zu >> 16
            zl = zu & 0x0000FFFF
            rxh = rxu >> 16
            rxl = rxu & 0x0000FFFF
            ryh = ryu >> 16
            ryl = ryu & 0x0000FFFF
            rzh = rzu >> 16
            rzl = rzu & 0x0000FFFF
            print("Moving to position: x=%s, y=%s, z=%s, rx=%s, ry=%s, rz=%s" % (x,y,z,rx,ry,rz))
            self.c.write_single_register(0x0330, xl)
            self.c.write_single_register(0x0331, xh)
            self.c.write_single_register(0x0332, yl)
            self.c.write_single_register(0x0333, yh)
            self.c.write_single_register(0x0334, zl)
            self.c.write_single_register(0x0335, zh)
            self.c.write_single_register(0x0336, rxl)
            self.c.write_single_register(0x0337, rxh)
            self.c.write_single_register(0x0338, ryl)
            self.c.write_single_register(0x0339, ryh)
            self.c.write_single_register(0x033A, rzl)
            self.c.write_single_register(0x033B, rzh)
            self.c.write_single_register(0x033E, 0)
            self.c.write_single_register(0x0324, speed)
            self.c.write_single_register(0x300, 301)
            self.waitForEndMove([x,y,z,rx,ry,rz])

    def sendArcMove(self, p1, p2):
        """With this function it's possible to do a arc move. The arm will move to p2 through p1.
        
        Arguments:
            p1 {list} -- 1st position [x,y,z,rx,ry,rz]
            p2 {list} -- 2nd position [x,y,z,rx,ry,rz]
        """
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")

        if self.c.is_open():
            x1 = int(np.binary_repr(p1[0], width=16), 2)
            y1 = int(np.binary_repr(p1[1], width=16), 2)
            z1 = int(np.binary_repr(p1[2], width=16), 2)
            rx1 = int(np.binary_repr(p1[3], width=16), 2)
            ry1 = int(np.binary_repr(p1[4], width=16), 2)
            rz1 = int(np.binary_repr(p1[5], width=16), 2)
            x2 = int(np.binary_repr(p2[0], width=16), 2)
            y2 = int(np.binary_repr(p2[1], width=16), 2)
            z2 = int(np.binary_repr(p2[2], width=16), 2)
            rx2 = int(np.binary_repr(p2[3], width=16), 2)
            ry2 = int(np.binary_repr(p2[4], width=16), 2)
            rz2 = int(np.binary_repr(p2[5], width=16), 2)
            self.c.write_single_register(0x1000, x1)
            self.c.write_single_register(0x1001, y1)
            self.c.write_single_register(0x1002, z1)
            self.c.write_single_register(0x1003, rx1)
            self.c.write_single_register(0x1004, ry1)
            self.c.write_single_register(0x1005, rz1)
            self.c.write_single_register(0x1006, x2)
            self.c.write_single_register(0x1007, y2)
            self.c.write_single_register(0x1008, z2)
            self.c.write_single_register(0x1009, rx2)
            self.c.write_single_register(0x100A, ry2)
            self.c.write_single_register(0x100B, rz2)
            self.c.write_single_register(0x0220, 4)
            self.c.write_single_register(0x0228, 6)
            print("Moving arc through %s to %s" % (p1, p2))
            self.waitForEndMove(p2)

    def jogRobot(self, direction, stop=False):
        """With this function it's possible to jog the robot in a certain direction.
        
        Arguments:
            direction {string} -- "X+", "Y+", "Z+", "RX+", "RY+", "RZ+", "X-", "Y-", "Z-", "RX-", "RY-", "RZ-"
        
        Keyword Arguments:
            stop {bool} -- if True the robot will stop jogging (default: {False})
        """
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")

        if self.c.is_open():
            self.c.write_single_register(0x033E, 0)
            self.c.write_single_register(0x0324, 80)
            if stop == True:
                self.c.write_single_register(0x0300, 0)
            elif direction == "X+":
                self.c.write_single_register(0x0300, 601)
            elif direction == "X-":
                self.c.write_single_register(0x0300, 602)
            elif direction == "Y+":
                self.c.write_single_register(0x0300, 603)
            elif direction == "Y-":
                self.c.write_single_register(0x0300, 604)
            elif direction == "Z+":
                self.c.write_single_register(0x0300, 605)
            elif direction == "Z-":
                self.c.write_single_register(0x0300, 606)
            elif direction == "RX+":
                self.c.write_single_register(0x0300, 607)
            elif direction == "RX-":
                self.c.write_single_register(0x0300, 608)
            elif direction == "RY+":
                self.c.write_single_register(0x0300, 609)
            elif direction == "RY-":
                self.c.write_single_register(0x0300, 610)
            elif direction == "RZ+":
                self.c.write_single_register(0x0300, 611)
            elif direction == "RZ-":
                self.c.write_single_register(0x0300, 612)

    def goHome(self):
        """This function moves the robot to the home position that is set in the robot software.
        !!!The robot needs to be enables for this function to work!!!
        """
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")

        if self.c.is_open():
            self.c.write_single_register(0x0300, 1405)
            print("Arm is homing...")
            #self.waitForEndMove()

    def writeDigitalOutput(self, output, state):
        """With this function it's possible to control the user digital outputs of the robot.
        
        Arguments:
            output {int} -- [The user digital output number that needs to be used [1 to 12]
            state {boolean} -- [The state that the output needs to be [True=HIGH (24V), False=LOW (0V)]]
        """
        if state == True:
            self.bits = self.bits | (1 << output-1)
        elif state == False:
            self.bits = self.bits & (0 << output-1)

        self.b = format(self.bits, 'b').zfill(16)
        print(self.b)
        self.c.write_single_register(0x02FE, int(self.b, 2))

    def getUserDigitalInputs(self):
        """With this function it's possible to get the states of all user digital inputs.
        
        Returns:
            [list] -- [Containing all states of the digital inputs [0 or 1]]
        """
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")

        if self.c.is_open():
            di = list(map(int, "{0:b}".format(self.c.read_holding_registers(0x02FA, 2)[0])))[::-1]
            l = 24 - len(di)
            for _ in range(0, l):
                di.append(0)
            print(di)
            return di

    def getUserDigitalOutputs(self):
        """With this function it's possible to get the states of all user digital outputs.
        
        Returns:
            [list] -- [Containing all states of the digital outputs [0 or 1]]
        """
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")

        if self.c.is_open():
            do = list(map(int, "{0:b}".format(self.c.read_holding_registers(0x02FC, 2)[0])))[::-1]
            l = 12 - len(do)
            for _ in range(0, l):
                do.append(0)
            print(do)
            return do

    def getToolPosition(self):
        """With this function it's possible to get the tool position.
        
        Returns:
            [list] -- [x,y,z,rx,ry,rz] in mm / degrees
        """
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")

        if self.c.is_open():
            x_r = self.c.read_holding_registers(0x00F0, 2)
            x = struct.unpack('i', struct.pack('HH', x_r[0], x_r[1]))[0]/1000
            y_r = self.c.read_holding_registers(0x00F2, 2)
            y = struct.unpack('i', struct.pack('HH', y_r[0], y_r[1]))[0]/1000
            z_r = self.c.read_holding_registers(0x00F4, 2)
            z = struct.unpack('i', struct.pack('HH', z_r[0], z_r[1]))[0]/1000
            rx_r = self.c.read_holding_registers(0x00F6, 2)
            rx = struct.unpack('i', struct.pack('HH', rx_r[0], rx_r[1]))[0]/1000
            ry_r = self.c.read_holding_registers(0x00F8, 2)
            ry = struct.unpack('i', struct.pack('HH', ry_r[0], ry_r[1]))[0]/1000
            rz_r = self.c.read_holding_registers(0x00FA, 2)
            rz = struct.unpack('i', struct.pack('HH', rz_r[0], rz_r[1]))[0]/1000
            return [x,y,z,rx,ry,rz]

    def getJointPositions(self):
        """With this function it's possible to get the joint positions
        
        Returns:
            [list] -- [1, 2, 3, 4, 5, 6] joint angles in radians
        """
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")

        if self.c.is_open():
            j1 = self.c.read_holding_registers(0x0168, 2)
            j1_a = struct.unpack('i', struct.pack('HH', j1[0], j1[1]))[0]
            j1_angle = radians(j1_a/1000)
            j2 = self.c.read_holding_registers(0x016A, 2)
            j2_a = struct.unpack('i', struct.pack('HH', j2[0], j2[1]))[0]
            j2_angle = radians(j2_a/1000)
            j3 = self.c.read_holding_registers(0x016C, 2)
            j3_a = struct.unpack('i', struct.pack('HH', j3[0], j3[1]))[0]
            j3_angle = radians(j3_a/1000)
            j4 = self.c.read_holding_registers(0x016E, 2)
            j4_a = struct.unpack('i', struct.pack('HH', j4[0], j4[1]))[0]
            j4_angle = radians(j4_a/1000)
            j5 = self.c.read_holding_registers(0x0150, 2)
            j5_a = struct.unpack('i', struct.pack('HH', j5[0], j5[1]))[0]
            j5_angle = radians(j5_a/1000)
            j6 = self.c.read_holding_registers(0x0152, 2)
            j6_a = struct.unpack('i', struct.pack('HH', j6[0], j6[1]))[0]
            j6_angle = radians(j6_a/1000)
            return [j1_angle, j2_angle, j3_angle, j4_angle, j5_angle, j6_angle]

    def saveToolPose(self, name, pose):
        """With this function it's possible to save a position to the poses.yaml yaml file.
        
        Arguments:
            name {string} -- Position name
            pose {[type]} -- [x,y,z,rx,ry,rz] position in mm / degrees
        
        Returns:
            [type] -- [description]
        """
        f = open('/home/%s/catkin_ws/src/delta/arm_driver/yaml/poses.yaml' % self.name, 'r')
        d = yaml.load(f)
        try:
            for n in d:
                if n == name:
                    f.close()
                    print("Pose %s already exists, delete the old pose and try again" % name)
                    return False
        except TypeError:
            pass
        f = open('/home/%s/catkin_ws/src/delta/arm_driver/yaml/poses.yaml' % self.name, 'a')
        d = {name: pose}
        yaml.dump(d, f, default_flow_style=False)
        f.close()
        print("Pose %s is saved" % name)
        return True

    def teachCurrentToolPose(self, msg):
        """With this function it's possible to save the current robot position to the poses.yaml yaml file.
        
        Arguments:
            msg {teach_position.srv} -- Name = position name
        
        Returns:
            [bool] -- True if succesfull
        """
        return self.saveToolPose(msg.name, self.getToolPosition())

    def getSavedToolPose(self, name):
        """Load a position from the poses.yaml yaml file.
        
        Arguments:
            name {string} -- Position name to load
        
        Returns:
            [list] -- [x,y,z,rx,ry,rz] position in mm /degrees
        """
        f = open('/home/%s/catkin_ws/src/delta/arm_driver/yaml/poses.yaml' % self.name, 'r')
        d = yaml.load(f)
        f.close()
        try:
            return d.get(name)
        except AttributeError:
            return "Position does not exist"

    def resetUserDigitalOutputs(self):
        """With this function it's possible to reset all user digital outputs to 0.
        """
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")

        if self.c.is_open():
            self.c.write_single_register(0x02FE, 0)

    def resetErrors(self, msg):
        """With this function it's possible to reset all errors.
        !!!ONLY USE THIS FUNCTION WHEN IT'S SAFE!!!
        """
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")
                return False

        if self.c.is_open():
            self.c.write_single_register(0x0180, 0xFFFF)
            time.sleep(0.1)
            self.c.write_single_register(0x0180, 0x0000)
            return True

    def waitForEndMove(self, pos):
        done = 0
        while done == 0:
            if self.getToolPosition() == pos and self.c.read_holding_registers(0x00E0, 1)[0] == 0:
                done = 1
        print("Position reached")
        time.sleep(1)

    def powerCallback(self, msg):
        if msg.on == True:
            self.enableRobot()
            return True
        elif msg.on == False:
            self.disableRobot()
            return True
        else:
            return False

if __name__ == "__main__":
    Robot = DRV90L()
    while not rospy.is_shutdown():
        Robot.sendPositionMove(252, 0, 776, -178, -90, -3, 100, "world")
        time.sleep(2)
        Robot.sendPositionMove(251, 0, 717, -178, -90, -3, 100, "world")
        time.sleep(2)