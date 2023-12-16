import pygame
from pygame.locals import *
from time import sleep
import time
import numpy as np
import lcm
import sys
sys.path.append('/usr/lib/python3.9/site-packages/')
from mbot_lcm_msgs.joy_t import joy_t

address = "udpm://239.255.76.67:7667?ttl=0"

lc = lcm.LCM(address)
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick found. Make sure your Bluetooth controller is connected.")
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

reference = {

}

try:
    while True:
        for event in pygame.event.get():
            # print(event)
            lcm_message = joy_t()
            if event.type == pygame.JOYAXISMOTION:
                # Read joystick axis data
                lcm_message.left_analog_X = joystick.get_axis(0)
                lcm_message.left_analog_Y = -joystick.get_axis(1)
                lcm_message.right_analog_X = joystick.get_axis(2)
                lcm_message.right_analog_Y = -joystick.get_axis(3)
                # Send the data to LCM
                lc.publish("MBOT_JOY", lcm_message.encode())

            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    lcm_message.button_A = 1
                elif event.button == 1:
                    lcm_message.button_B = 1
                elif event.button == 4:
                    lcm_message.button_Y = 1
                elif event.button == 3:
                    lcm_message.button_X = 1
                lc.publish("MBOT_JOY", lcm_message.encode())
            
            elif event.type == pygame.JOYBUTTONUP:
                if event.button == 0:
                    lcm_message.button_A = 0
                elif event.button == 1:
                    lcm_message.button_B = 0
                elif event.button == 4:
                    lcm_message.button_Y = 0
                elif event.button == 3:
                    lcm_message.button_X = 0
                lc.publish("MBOT_JOY", lcm_message.encode())

            # Not available till Pygame 2.x is installed. In Pygame 1.9 there exist no event to detect controller disconnect.

            # elif event.type == pygame.JOYDEVICEREMOVED:
            #     joystick.quit()
            #     print("Joystick Disconnected")

            # elif event.type == pygame.JOYDEVICEADDED:
            #     joystick.init()
            #     print("Joystick Connected")


except Exception as e: 
    print("oh no... you broke it")
    print(e)
