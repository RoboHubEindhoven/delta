#!/usr/bin/env python

import serial
from time import sleep

class EndEffector():
    def __init__(self):
        self.arduino = serial.Serial('/dev/ttyUSB0', 9600)
        
    def sendCommand(self, val):
        self.arduino.write(str(chr(val)))

    def startBrushing(self):
        self.sendCommand(1)

    def stopBrushing(self):
        self.sendCommand(0)

    def startDispensing(self):
        self.sendCommand("StartDispensing")

    def stopDispensing(self):
        self.sendCommand("StopDispensing")

if __name__ == "__main__":
    EE = EndEffector()
    while True:
        EE.startBrushing()
        sleep(5)
        EE.stopBrushing()
        sleep(5)