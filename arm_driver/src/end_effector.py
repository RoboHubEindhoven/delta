#!/usr/bin/env python

import serial
from time import sleep

class EndEffector():
    """The "EndEffector" class contains functions to control the end effector
    """
    def __init__(self):
        self.arduino = serial.Serial('/dev/ttyACM0', 115200)
        
    def sendCommand(self, val):
        """Function to send values to the arduino over serial communication.
        
        Arguments:
            val {int} -- Value to send to arduino
        """
        self.arduino.write(str(val))

    def startDispensing(self):
        """Function to start dispensing liquid.
        """
        self.sendCommand(2)

    def stopDispensing(self):
        """Function to stop dispensing liquid.
        """
        self.sendCommand(3)

    def startBrushing(self):
        """Function to start rotating the brush.
        """
        self.sendCommand(1)

    def stopBrushing(self):
        """Function to stop rotating the brush.
        """
        self.sendCommand(0)

if __name__ == "__main__":
    EE = EndEffector()
    #while True:
    #    EE.startBrushing()
    #    print("Start brushing")
    #    sleep(1)
    #    EE.startDispensing()
    #    print("Start dispensing")
    #    sleep(5)
    #    EE.stopBrushing()
    #    EE.stopDispensing()
    #    print("Stop brushing")
    #    print("Stop dispensing")
    #    sleep(1)