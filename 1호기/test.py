#!/usr/bin/env python3
# encoding: utf-8
import rospy
import random
import threading
import time
from time import sleep
from jetbotmini_msgs.msg import *
from jetbotmini_msgs.srv import *
import RPi.GPIO as GPIO
import smbus
import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

bus = smbus.SMBus(1)
ADDRESS = 0x1B

Key1_pin = 8
GPIO.setmode(GPIO.BCM)
GPIO.setup(Key1_pin, GPIO.IN)

print("Starting now!")

class transbot_driver:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.srv_Buzzer = rospy.Service("/Buzzer", Buzzer, self.Buzzercallback)
        self.srv_Motor = rospy.Service("/Motor", Motor, self.Motorcallback)
        self.motorcontrol = rospy.Service("/MotorControl", String, self.Motorcontrol)
 
        # self.volPublisher = rospy.Publisher("/voltage", Battery, queue_size=10)

    def cancel(self):
        self.srv_Buzzer.shutdown()
        self.srv_Motor.shutdown()
        # self.volPublisher.unregister()
        GPIO.cleanup()
        rospy.loginfo("Close the robot...")
        rospy.sleep(1)

    # def pub_data(self):
    #     # Release trolley battery voltage
    #     while not rospy.is_shutdown():
    #         sleep(30)
    #         AD_value = bus.read_i2c_block_data(ADDRESS,0x00,2)
    #         voltage = ((AD_value[0] << 8) + AD_value[1]) * 13.3 / 1023.0
    #         battery = Battery()
    #         battery.Voltage = voltage
    #         self.volPublisher.publish(battery)

    def Buzzercallback(self, request):
        # Buzzer control, server callback function
        if not isinstance(request, BuzzerRequest): return
        bus.write_byte_data(ADDRESS,0x06,request.buzzer)
        sleep(0.01)
        bus.write_byte_data(ADDRESS,0x06,request.buzzer)
        response = BuzzerResponse()
        response.result = True
        return response

    def Motorcallback(self, request):
        # Motor control, server callback function
        if not isinstance(request, MotorRequest): return
        if request.leftspeed < 0:
            request.leftspeed = -request.leftspeed
            leftdir = 0
        else:
            leftdir = 1
        if request.rightspeed < 0:
            request.rightspeed = -request.rightspeed
            rightdir = 0
        else:
            rightdir = 1
        bus.write_i2c_block_data(ADDRESS,0x01,[leftdir,int(request.leftspeed*255),rightdir,int(request.rightspeed*255)])
        sleep(0.01)
        bus.write_i2c_block_data(ADDRESS,0x01,[leftdir,int(request.leftspeed*255),rightdir,int(request.rightspeed*255)])
        response = MotorResponse()
        response.result = True
        return response
    
    def Motorcontrol(self, request): 
        # # Handle motor control request based on String input
        # if not isinstance(request, String): 
        #     return String(data="Invalid request type")


        command = request.data.lower()

        # Default values
        speed = 0.5
        leftdir = 1
        rightdir = 1

        # Process movement commands
        if command == "forward":
            leftdir = rightdir = 1
        elif command == "backward":
            leftdir = rightdir = 0
        elif command == "left":
            leftdir = 0
            rightdir = 1
        elif command == "right":
            leftdir = 1
            rightdir = 0
        elif command == "stop":
            speed = 0  # Stop the motors
        elif "speed" in command:
            if "fast" in command:
                speed = 0.8
            elif "slow" in command:
                speed = 0.3
            elif "normal" in command:
                speed = 0.5

        # Send motor control command via I2C
        bus.write_i2c_block_data(ADDRESS, 0x01, [
            leftdir, int(speed * 255), rightdir, int(speed * 255)
        ])

if __name__ == '__main__':
    rospy.init_node("driver_node", anonymous=False)
    try:
        driver = transbot_driver()
        driver.pub_data()
        rospy.spin()
    except Exception as e:
        rospy.loginfo(e)
        rospy.loginfo("Final!!!")





