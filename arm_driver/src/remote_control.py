#!/usr/bin/env python

from send_commands import Sender
from arm_driver.srv import reset_errors
import pygame
import os
import time
import serial

class PS4_Controller():
    def __init__(self):
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        self.INACTIVITY_RECONNECT_TIME = 5
        self.RECONNECT_TIMEOUT = 1
        self.lastActive = 0
        self.lastTime = 0
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.axis_data = {}
        self.button_data = {}
        self.robot = Sender()
        
    def listen(self):
        try:
            msg = reset_errors()
            while True:
                events = pygame.event.get()
                for event in events:
                    if event.type == pygame.JOYAXISMOTION or event.type == pygame.JOYBUTTONUP or event.type == pygame.JOYBUTTONDOWN:
                        if event.type == pygame.JOYBUTTONDOWN:
                            self.button_data[event.button] = True
                        if event.type == pygame.JOYBUTTONUP:
                            self.button_data[event.button] = False
                        if event.type == pygame.JOYAXISMOTION:
                            self.axis_data[event.axis] = round(event.value,2)

                        if not self.hasController():
                            self.robot.jogRobot(None, stop=True)
                            print("Reconnect PS4 controller...")
                        elif self.axis_data.get(0) == -1.0:
                            self.robot.jogRobot("Y+")
                            print("Moving in Y+ direction")
                        elif self.axis_data.get(0) == 1.0:
                            self.robot.jogRobot("Y-")
                            print("Moving in Y- direction")
                        elif self.axis_data.get(1) == -1.0:
                            self.robot.jogRobot("X+")
                            print("Moving in X+ direction")
                        elif self.axis_data.get(1) == 1.0:
                            self.robot.jogRobot("X-")
                            print("Moving in X- direction")
                        elif self.axis_data.get(2) == 1.0:
                            self.robot.jogRobot("Z-")
                            print("Moving in Z- direction")
                        elif self.axis_data.get(3) == -1.0:
                            self.robot.jogRobot("RY+")
                            print("Moving in RY+ direction")
                        elif self.axis_data.get(3) == 1.0:
                            self.robot.jogRobot("RY-")
                            print("Moving in RY- direction")
                        elif self.axis_data.get(4) == -1.0:
                            self.robot.jogRobot("RX+")
                            print("Moving in RX+ direction")
                        elif self.axis_data.get(4) == 1.0:
                            self.robot.jogRobot("RX-")
                            print("Moving in RX- direction")
                        elif self.axis_data.get(5) == 1.0:
                            self.robot.jogRobot("Z+")
                            print("Moving in Z+ direction")
                        elif self.button_data.get(4) == True:
                            self.robot.jogRobot("RZ-")
                            print("Moving in RZ- direction")
                        elif self.button_data.get(5) == True:
                            self.robot.jogRobot("RZ+")
                            print("Moving in RZ+ direction")
                        elif self.button_data.get(9) == True:
                            self.robot.resetErrors(msg)
                            self.button_data[9] = False
                            print("Resetting errors")
                        elif self.button_data.get(8) == True:
                            self.robot.disableRobot()
                            self.button_data[8] = False
                            print("Robot disabled")
                        elif self.button_data.get(0) == True:
                            self.robot.enableRobot()
                            self.button_data[0] = False
                            print("Robot enabled")
                        elif self.button_data.get(10) == True:
                            self.robot.goHome()
                            self.button_data[10] = False
                            print("Homing robot")
                        else:
                            self.robot.jogRobot(None, stop=True)
                            #print("Jogging stopped")
                        
                        self.lastActive = time.time()

        except KeyboardInterrupt:
            print("EXITING NOW")
            self.controller.quit()

    def hasController(self):
        now = time.time()
        if now - self.lastActive > self.INACTIVITY_RECONNECT_TIME and now - self.lastTime > self.RECONNECT_TIMEOUT:
            self.lastTime = now
            pygame.joystick.quit()
            pygame.joystick.init()
        return pygame.joystick.get_count() > 0

if __name__ == "__main__":
    ps4 = PS4_Controller()
    ps4.listen()
