#!/usr/bin/env python

from pyModbusTCP.client import ModbusClient
import time

class Sender():
    def __init__(self):
        self.c = ModbusClient(host="192.168.1.1", auto_open=True, auto_close=False, port=502, debug=False, unit_id=2)
        self.bits = 0

    def sendMove(self, x, y, z, rx, ry, rz, speed, frame):
        xu = int(x * 1000)
        yu = int(y * 1000)
        zu = int(z * 1000)
        rxu = int(rx * 1000)
        ryu = int(ry * 1000)
        rzu = int(rz * 1000)
        print(xu, yu, zu, rxu, ryu, rzu, speed)

        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect/nTrying to connect...")

        if self.c.is_open():
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

    def goHome(self):
        if not self.c.is_open():
            if not self.c.open():
                print("Unable to connect/nTrying to connect...")

        if self.c.is_open():
            self.c.write_single_register(0x0300, 1405)
            self.waitForEndMove()

    def writeDigitalOutput(self, output, state):
        if state == True:
            self.bits = self.bits | (1 << output-1)
        elif state == False:
            self.bits = self.bits & (0 << output-1)

        self.b = format(self.bits, 'b').zfill(16)
        print(self.b)
        self.c.write_single_register(0x02FE, int(self.b, 2))

    def waitForEndMove(self):
        time.sleep(1)
        while self.c.read_holding_registers(0x00E0, 1)[0] == 1:
            #print("Robot is moving to position")
            pass
        time.sleep(1)

if __name__ == "__main__":
    s = Sender()
    while True:
        s.sendMove(400, 150, 850, 180, 0, 0, 100, 'world')
        s.sendMove(150, 0, 800, 180, 0, 90, 100, 'world')
        s.sendMove(400, -150, 900, 180, 0, 0, 100, 'world')
        s.goHome()