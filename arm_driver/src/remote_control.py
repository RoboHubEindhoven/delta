#!/usr/bin/env python

from send_commands import Sender
import pygame

class PS4_Controller():
    def __init__(self):
        pygame.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.axis_data = {}
        self.button_data = {}
        self.robot = Sender()
        self.robot.enableRobot()
        
    def listen(self):
        try:
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
                
                        if self.axis_data.get(0) == -1.0:
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
                        elif self.button_data.get(10) == True:
                            self.robot.goHome()
                            self.button_data[10] = False
                            print("Homing robot")
                        else:
                            self.robot.jogRobot(None, stop=True)
                            print("Jogging stopped")
                        

        except KeyboardInterrupt:
            print("EXITING NOW")
            self.controller.quit()

if __name__ == "__main__":
    ps4 = PS4_Controller()
    ps4.listen()