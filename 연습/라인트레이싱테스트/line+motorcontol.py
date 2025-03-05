#!/usr/bin/env python3
# encoding: utf-8
import rospy
import smbus
import cv2 as cv
import numpy as np
from time import sleep
from jetbotmini_msgs.srv import *
import time
from PIL import Image, ImageDraw, ImageFont
import os
import threading
from time import sleep
from jetbotmini_msgs.msg import *
from jetbotmini_msgs.srv import *
from actionlib_msgs.msg import GoalID
from dynamic_reconfigure.server import Server

global color_lower
global color_upper
global show
global mot_start

bus = smbus.SMBus(1)
ADDRESS = 0x1B
mot_start = 0
# gstreamer pipeline for camera capture
def gstreamer_pipeline(
    capture_width=320,
    capture_height=240,
    display_width=320,
    display_height=240,
    framerate=5,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

class TransbotDriver:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.srv_Buzzer = rospy.Service("/Buzzer", Buzzer, self.Buzzercallback)
        self.motorcontrol = rospy.Service("/MotorControl", MotorControl, self.Motorcontrol)

        # Color range for detecting line (HSV range)
        self.Hmin = 100
        self.Smin = 43
        self.Vmin = 46
        self.Hmax = 124
        self.Smax = 255
        self.Vmax = 255

        # Speed setting for motors
        self.speed_value = 0.3
        self.capture = cv.VideoCapture(gstreamer_pipeline(flip_method=0), cv.CAP_GSTREAMER)

        if not self.capture.isOpened():
            rospy.logerr("Failed to open camera.")
        else:
            self.start_camera_stream()
    def start_camera_stream(self):
        
        global mot_start
        rate = rospy.Rate(30)  # Process at 30Hz
        while not rospy.is_shutdown():
            ret, frame = self.capture.read()
            if not ret:
                continue
            processed_frame, mask = self.colorDisplay(frame)

            # Show processed frame using OpenCV
            cv.imshow("Processed Frame", processed_frame)

            # Check for key press
            action = cv.waitKey(1) & 0xFF

            # If 'q' is pressed, stop the robot and close the window
            if action == ord('q'):
                self.stop_motors()  # Stop motors on exit
                break
            # If 's' is pressed, set mot_start to 1
            elif action == ord('s'):
                mot_start = 1
                rospy.loginfo("mot_start set to 1")

            rate.sleep()
            
    def colorDisplay(self, image):
        global mot_start
        # Convert the image to HSV
        color_lower = np.array([self.Hmin, self.Smin, self.Vmin], dtype=np.uint8)
        color_upper = np.array([self.Hmax, self.Smax, self.Vmax], dtype=np.uint8)

        # Pre-process image
        image = cv.GaussianBlur(image, (5, 5), 0)
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        hsv = hsv.astype(np.uint8)
        mask = cv.inRange(hsv, color_lower, color_upper)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)
        mask = cv.GaussianBlur(mask, (3, 3), 0)
        
        # Find contours
        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]

        if len(cnts) > 0:
            # Find the largest contour
            cnt = max(cnts, key=cv.contourArea)
            (color_x, color_y), color_radius = cv.minEnclosingCircle(cnt)
            
            if color_radius > 30:  # Only consider sufficiently large contours
                cv.circle(image, (int(color_x), int(color_y)), int(color_radius), (255, 0, 255), 2)
                color_x = (160 - color_x) / 160 # Calculate error based on center of the frame
                
                    
                if mot_start == 1:
                    self.speed_value = 0.3
                    self.turn_value = 0.2

                    # Calculate left and right motor speeds based on error
                    left_speed = int((self.speed_value - self.turn_value * color_x) * 255)  # Convert to 0-255 range
                    right_speed = int((self.speed_value + self.turn_value * color_x) * 255)

                    # Ensure speeds are within valid range (0-255)
                    left_speed = max(0, min(255, left_speed))
                    right_speed = max(0, min(255, right_speed))

                    # Log calculated speeds

                    # Send motor control signals via I2C
                    try:
                        bus.write_i2c_block_data(ADDRESS, 0x01, [1, left_speed, 1, right_speed])  # Forward motion
                    except Exception as e:
                        rospy.logerr(f"Failed to send motor command: {e}")

            else:
                # No contours found, stop motors
                bus.write_i2c_block_data(ADDRESS, 0x01, [1, 0, 1, 0])  # Stop both motors
                rospy.loginfo("Motors stopped.")
        else:
            # No contours found, stop motors
            bus.write_i2c_block_data(ADDRESS, 0x01, [1, 0, 1, 0])  # Stop both motors
            rospy.loginfo("Motors stopped.")

        return image, mask

    def cancel(self):
        self.srv_Buzzer.shutdown()
        rospy.loginfo("Closing the robot...")
        rospy.sleep(1)

    


    def stop_motors(self):
        # Stop the motors
        bus.write_i2c_block_data(ADDRESS, 0x01, [1, 0, 1, 0])  # Stop both motors
        rospy.loginfo("Motors stopped.")

    def Buzzercallback(self, request):
        if not isinstance(request, BuzzerRequest): return
        bus.write_byte_data(ADDRESS, 0x06, request.buzzer)
        sleep(0.01)
        bus.write_byte_data(ADDRESS, 0x06, request.buzzer)
        return BuzzerResponse(result=True)

    def Motorcontrol(self, request):
        speed = 0.5
        leftdir, rightdir = 1, 1

        if request.command == "forward":
            leftdir = rightdir = 1
        elif request.command == "backward":
            leftdir = rightdir = 0
        elif request.command == "left":
            leftdir = 0
            rightdir = 1
        elif request.command == "right":
            leftdir = 1
            rightdir = 0
        elif request.command == "stop":
            speed = 0
        elif "speed" in request.command:
            speed = {"fast": 0.8, "slow": 0.3, "normal": 0.5}.get(request.command.split()[-1], 0.5)

        # Send motor control commands
        try:
            bus.write_i2c_block_data(ADDRESS, 0x01, [leftdir, int(speed*255), rightdir, int(speed*255)])
            sleep(0.01)
            bus.write_i2c_block_data(ADDRESS, 0x01, [leftdir, int(speed*255), rightdir, int(speed*255)])
            rospy.loginfo("Motor control successful.")
        except Exception as e:
            rospy.logerr("Motor control failed: %s", e)

if __name__ == '__main__':
    rospy.init_node("driver_node", anonymous=False)
    try:
        driver = TransbotDriver()
        rospy.spin()
    except Exception as e:
        rospy.loginfo(e)
        rospy.loginfo("Final!!!")