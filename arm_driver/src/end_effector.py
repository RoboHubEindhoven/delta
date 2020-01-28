#!/usr/bin/env python

import serial
from time import sleep

class EndEffector():
    def __init__(self):
        self.arduino = serial.Serial('/dev/ttyACM0', 115200)
        
    def sendCommand(self, val):
        self.arduino.write(str(val))

    def startDispensing(self):
        self.sendCommand(2)
        sleep(0.1)

    def stopDispensing(self):
        self.sendCommand(3)
        sleep(0.1)

    def startBrushing(self):
        self.sendCommand(1)
        sleep(0.1)

    def stopBrushing(self):
        self.sendCommand(0)
        sleep(0.1)

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