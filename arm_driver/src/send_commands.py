#!/usr/bin/env python

from pyModbusTCP.client import ModbusClient
import time
import numpy as np

class Sender():
    """The "Sender" class contains methods to control the robot.
    """
    def __init__(self):
        self.c = ModbusClient(host="192.168.1.1", auto_open=True, auto_close=False, port=502, debug=False, unit_id=2)
        self.bits = 0

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

    def sendMove(self, x, y, z, rx, ry, rz, speed, frame):
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
            self.waitForEndMove()

    def jogRobot(self, direction, stop=False):
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")

        if self.c.is_open():
            if stop == True:
                self.c.write_single_register(0x0259, 0x0000)
                self.c.write_single_register(0x025A, 0x0000)
                self.c.write_single_register(0x025B, 0x0000)
                self.c.write_single_register(0x025C, 0x0000)
                self.c.write_single_register(0x025D, 0x0000)
                self.c.write_single_register(0x025E, 0x0000)
                self.c.write_single_register(0x025F, 0x0000)
                self.c.write_single_register(0x0260, 0x0000)
                self.c.write_single_register(0x0261, 0x0000)
                self.c.write_single_register(0x0262, 0x0000)
                self.c.write_single_register(0x0263, 0x0000)
                self.c.write_single_register(0x0264, 0x0000)
            elif direction == "X+":
                self.c.write_single_register(0x0259, 0xFFFF)
            elif direction == "X-":
                self.c.write_single_register(0x025A, 0xFFFF)
            elif direction == "Y+":
                self.c.write_single_register(0x025B, 0xFFFF)
            elif direction == "Y-":
                self.c.write_single_register(0x025C, 0xFFFF)
            elif direction == "Z+":
                self.c.write_single_register(0x025D, 0xFFFF)
            elif direction == "Z-":
                self.c.write_single_register(0x025E, 0xFFFF)
            elif direction == "RX+":
                self.c.write_single_register(0x025F, 0xFFFF)
            elif direction == "RX-":
                self.c.write_single_register(0x0260, 0xFFFF)
            elif direction == "RY+":
                self.c.write_single_register(0x0261, 0xFFFF)
            elif direction == "RY-":
                self.c.write_single_register(0x0262, 0xFFFF)
            elif direction == "RZ+":
                self.c.write_single_register(0x0263, 0xFFFF)
            elif direction == "RZ-":
                self.c.write_single_register(0x0264, 0xFFFF)

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
            self.waitForEndMove()

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

    def resetUserDigitalOutputs(self):
        """With this function it's possible to reset all user digital outputs to 0.
        """
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")

        if self.c.is_open():
            self.c.write_single_register(0x02FE, 0)

    def resetErrors(self):
        """With this function it's possible to reset all errors.
        !!!ONLY USE THIS FUNCTION WHEN YOU'RE IT'S SAFE!!!
        """
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect\nTrying to connect...")

        if self.c.is_open():
            self.c.write_single_register(0x0180, 0xFFFF)
            time.sleep(0.1)
            self.c.write_single_register(0x0180, 0x0000)

    def waitForEndMove(self):
        time.sleep(1.5)
        while self.c.read_holding_registers(0x00E0, 1)[0] == 1:
            pass
        time.sleep(1.5)

if __name__ == "__main__":
    s = Sender()
    s.enableRobot()
    s.goHome()
    s.sendMove(400, 250, 850, 180, 0, 0, 100, 'world')
    s.sendMove(400, 0, 600, 90, 0, 0, 100, 'world')
    s.sendMove(100, -300, 900, 90, 0, 0, 100, 'world')
    s.disableRobot()


    #while True:
        #s.writeDigitalOutput(1, True)
        #s.sendMove(400, 150, 850, 180, 0, 0, 100, 'world')
        #s.sendMove(150, 0, 800, 180, 0, 90, 100, 'world')
        #s.writeDigitalOutput(6, True)
        #s.sendMove(400, -150, 900, 180, 0, 0, 100, 'world')
        #s.writeDigitalOutput(1, False)
        #s.goHome()
        #s.writeDigitalOutput(6, False)
