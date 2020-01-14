#!/usr/bin/env python

from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import time

class FT_Sensor():
    def __init__(self):
        self.sensor = ModbusClient(method='rtu', port='/dev/ttyUSB0', baudrate=19200, timeout=1, stopbits=1)
        self.sensor.connect()

    def getVals(self):
        vals = self.sensor.read_holding_registers(180, 6, unit = 0x0009)
        l = []
        for registers in vals.registers:
            l.append(twos_comp(registers, 16))
        return l

def twos_comp(val, bits):
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val

if __name__ == "__main__":
    s = FT_Sensor()
    while True:
        print(s.getVals())