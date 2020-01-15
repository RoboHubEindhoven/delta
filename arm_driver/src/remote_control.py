#!/usr/bin/env python

import pygame

class PS4_Controller():
    def __init__(self):
        pygame.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.axis_data = {}
        
    def listen(self):
        try:
            while True:
                events = pygame.event.get()
                for event in events:
                    if event.type == pygame.JOYBUTTONDOWN:
                        if self.controller.get_button(0):
                            print("X Pressed")
                        elif self.controller.get_button(1):
                            print("Circle Pressed")
                        elif self.controller.get_button(2):
                            print("Triangle Pressed")
                        elif self.controller.get_button(3):
                            print("Square Pressed")
                        elif self.controller.get_button(4):
                            print("L1 Pressed")
                        elif self.controller.get_button(5):
                            print("R1 Pressed")
                        elif self.controller.get_button(6):
                            print("L2 Pressed")
                        elif self.controller.get_button(7):
                            print("R2 Pressed")
                        elif self.controller.get_button(8):
                            print("SHARE Pressed")
                        elif self.controller.get_button(9):
                            print("OPTIONS Pressed")
                        elif self.controller.get_button(10):
                            print("Power Button Pressed")
                        elif self.controller.get_button(11):
                            print("Left Analog Pressed")
                        elif self.controller.get_button(12):
                            print("Right Analog Pressed")

                    elif event.type == pygame.JOYBUTTONUP:
                        print("Button Released")

                    elif event.type == pygame.JOYAXISMOTION:
                        self.axis_data[event.axis] = round(event.value,2)
                        print(self.axis_data.get(5))

        except KeyboardInterrupt:
            print("EXITING NOW")
            self.controller.quit()

if __name__ == "__main__":
    ps4 = PS4_Controller()
    ps4.listen()