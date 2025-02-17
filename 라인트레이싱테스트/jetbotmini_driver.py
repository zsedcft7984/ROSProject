#!/usr/bin/env python3
# encoding: utf-8
import rospy
import time
import subprocess
from time import sleep
from jetbotmini_msgs.msg import *
from jetbotmini_msgs.srv import *
import RPi.GPIO as GPIO
import smbus
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import cv2

bus = smbus.SMBus(1)
ADDRESS = 0x1B
# GStreamer 파이프라인 설정
def gstreamer_pipeline():
    
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), width=(int)640, height=(int)480, "
        "format=(string)NV12, framerate=(fraction)30/1 ! "
        "nvvidconv ! video/x-raw, format=(string)BGR ! "
        "videoconvert ! appsink"
        )
    

class transbot_driver:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.srv_Buzzer = rospy.Service("/Buzzer", Buzzer, self.Buzzercallback)
        self.srv_Motor = rospy.Service("/Motor", Motor, self.Motorcallback)
        self.srv_MotorControl = rospy.Service("/MotorControl", MotorControl, self.Motorcontrol)
        
        # 카메라 퍼블리셔 설정
        self.image_pub = rospy.Publisher("/cam", Image, queue_size=10)
        self.process = subprocess.Popen(
            gstreamer_pipeline(),
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        
        rospy.loginfo("Camera stream started...")

    def cancel(self):
        self.srv_Buzzer.shutdown()
        self.srv_Motor.shutdown()
        self.srv_MotorControl.shutdown()
        self.process.terminate()  # GStreamer 프로세스 종료
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
        rospy.loginfo("Starting GStreamer camera stream...")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                # GStreamer 출력에서 프레임을 읽음
                frame_data = self.process.stdout.read(640 * 480 * 3)  # 640x480 RGB 이미지
                if len(frame_data) != 640 * 480 * 3:
                    rospy.logwarn("Incomplete frame received")
                    continue

                # ROS Image 메시지 생성
                img_msg = Image()
                img_msg.header.stamp = rospy.Time.now()
                img_msg.height = 480
                img_msg.width = 640
                img_msg.encoding = "rgb8"
                img_msg.is_bigendian = 0
                img_msg.step = 640 * 3
                img_msg.data = frame_data

                self.image_pub.publish(img_msg)  # 영상 퍼블리싱
            except Exception as e:
                rospy.logerr(f"Error in video stream: {e}")

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("driver_node", anonymous=False)
    try:
        driver = transbot_driver()
        driver.start_stream()  # 카메라 스트리밍 시작
        rospy.spin()
    except Exception as e:
        rospy.loginfo(e)
        rospy.loginfo("Final!!!")
