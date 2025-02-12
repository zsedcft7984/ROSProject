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
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

# GStreamer 파이프라인 설정
def gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=30,
    flip_method=0):
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
            display_height
        )
    )

class transbot_driver:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.srv_Buzzer = rospy.Service("/Buzzer", Buzzer, self.Buzzercallback)
        self.srv_Motor = rospy.Service("/Motor", Motor, self.Motorcallback)
        self.srv_MotorControl = rospy.Service("/MotorControl", MotorControl, self.Motorcontrol)
        
        # 카메라 퍼블리셔 설정
        self.image_pub = rospy.Publisher("/cam", Image, queue_size=10)
        self.bridge = CvBridge()
        self.capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        
        if not self.capture.isOpened():
            rospy.logerr("Camera not found!")
            return
        
        rospy.loginfo("Camera stream started...")

    def cancel(self):
        self.srv_Buzzer.shutdown()
        self.srv_Motor.shutdown()
        self.srv_MotorControl.shutdown()
        GPIO.cleanup()
        rospy.loginfo("Close the robot...")
        rospy.sleep(1)

    def Buzzercallback(self, request):
        # Buzzer control
        if not isinstance(request, BuzzerRequest): return
        bus.write_byte_data(ADDRESS, 0x06, request.buzzer)
        sleep(0.01)
        bus.write_byte_data(ADDRESS, 0x06, request.buzzer)
        response = BuzzerResponse()
        response.result = True
        return response

    def Motorcallback(self, request):
        # Motor control
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
        bus.write_i2c_block_data(ADDRESS, 0x01, [leftdir, int(request.leftspeed*255), rightdir, int(request.rightspeed*255)])
        sleep(0.01)
        bus.write_i2c_block_data(ADDRESS, 0x01, [leftdir, int(request.leftspeed*255), rightdir, int(request.rightspeed*255)])
        response = MotorResponse()
        response.result = True
        return response
    
    def Motorcontrol(self, request): 
        # Default motor control
        speed = 0.5
        leftdir = 1
        rightdir = 1

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
            speed = 0  # Stop the motors
        elif "speed" in request.command:
            if "fast" in request.command:
                speed = 0.8
            elif "slow" in request.command:
                speed = 0.3
            elif "normal" in request.command:
                speed = 0.5

        # Send motor control command via I2C
        bus.write_i2c_block_data(ADDRESS, 0x01, [leftdir, int(speed*255), rightdir, int(speed*255)])
        sleep(0.01)
        bus.write_i2c_block_data(ADDRESS, 0x01, [leftdir, int(speed*255), rightdir, int(speed*255)])
    
    def start_stream(self):
        rospy.loginfo("Starting camera stream...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            start_time = time.time()
            ret, frame = self.capture.read()
            if not ret:
                rospy.logwarn("Failed to capture image")
                continue
            
            # OpenCV 이미지를 ROS 이미지로 변환
            ros_image = self.cv2_to_ros_image(frame)
            
            # 퍼블리시
            self.image_pub.publish(ros_image)
            
            # FPS 계산
            end_time = time.time()
            fps = 1.0 / (end_time - start_time)
            rospy.loginfo(f"Publishing image - FPS: {fps:.2f}")
            
            rate.sleep()

        self.capture.release()
    
    def cv2_to_ros_image(self, cv_image):
        """ OpenCV 이미지를 ROS 이미지로 변환 """
        ros_image = Image()

        # 헤더 설정
        ros_image.header.stamp = rospy.Time.now()

        # 이미지 타입 및 데이터 설정
        ros_image.height = cv_image.shape[0]
        ros_image.width = cv_image.shape[1]
        ros_image.encoding = "bgr8"  # OpenCV에서 BGR로 읽음
        ros_image.is_bigendian = False
        ros_image.step = cv_image.shape[1] * 3  # BGR의 경우 3 채널

        # 데이터 필드에 OpenCV 이미지 데이터를 넣음
        ros_image.data = np.array(cv_image).tobytes()

        return ros_image

if __name__ == '__main__':
    rospy.init_node("driver_node", anonymous=False)
    try:
        driver = transbot_driver()
        driver.start_stream()  # 카메라 스트리밍 시작
        rospy.spin()
    except Exception as e:
        rospy.loginfo(e)
        rospy.loginfo("Final!!!")
