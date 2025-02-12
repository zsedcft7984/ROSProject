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
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

bus = smbus.SMBus(1)
ADDRESS = 0x1B
Led_G_pin = 24
Led_B_pin = 23
Key1_pin = 8
GPIO.setmode(GPIO.BCM)
GPIO.setup(Led_G_pin, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(Led_B_pin, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(Key1_pin, GPIO.IN)

print("Starting now!")

class transbot_driver:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.srv_Buzzer = rospy.Service("/Buzzer", Buzzer, self.Buzzercallback)
        self.srv_Servo = rospy.Service("/Servo", Servo, self.Servocallback)
        self.srv_LEDBLUE = rospy.Service("/LEDBLUE", LEDBLUE, self.LEDBLUEcallback)
        self.srv_LEDGREE = rospy.Service("/LEDGREE", LEDGREE, self.LEDGREEcallback)
        self.srv_Motor = rospy.Service("/Motor", Motor, self.Motorcallback)
        # self.volPublisher = rospy.Publisher("/voltage", Battery, queue_size=10)
        self.qrPublisher = rospy.Publisher("/parsing", String, queue_size=10)
        self.motorSubscriber = rospy.Subscriber("/Motor", String, self.motor_callback)
        self.bridge = CvBridge()
        #rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Transbot Driver Initialized")

    def cancel(self):
        self.srv_Buzzer.shutdown()
        self.srv_Servo.shutdown()
        self.srv_LEDBLUE.shutdown()
        self.srv_LEDGREE.shutdown()
        self.srv_Motor.shutdown()
#         self.volPublisher.unregister()
        self.qrPublisher.unregister()
        GPIO.cleanup()
        rospy.loginfo("Close the robot...")
        rospy.sleep(1)

#     def pub_data(self):
#         # 发布小车电池电压
#         # Release trolley battery voltage
#         while not rospy.is_shutdown():
#             sleep(30)
#             AD_value = bus.read_i2c_block_data(ADDRESS,0x00,2)
#             voltage = ((AD_value[0] << 8) + AD_value[1]) * 13.3 / 1023.0
#             battery = Battery()
#             battery.Voltage = voltage
#             self.volPublisher.publish(battery)
            
    def pub_qrdata(self) :
        while not rospy.is_shutdown():
            sleep(5)
            self.qrPublisher.publish("hello, sungmin!")

    def motor_callback(self, msg) :
        rospy.loginfo(msg)
        if msg.data == "go" :
            leftdir = 1
            rightdir = 1
            leftspeed = 1
            rightspeed = 1
            bus.write_i2c_block_data(ADDRESS,0x01,[leftdir,int(leftspeed*255),rightdir,int(rightspeed*255)])
            sleep(0.01)
            bus.write_i2c_block_data(ADDRESS,0x01,[leftdir,int(leftspeed*255),rightdir,int(rightspeed*255)])
            
        else :
            leftdir = 1
            rightdir = 1
            leftspeed = 0
            rightspeed = 0
            bus.write_i2c_block_data(ADDRESS,0x01,[leftdir,int(leftspeed*255),rightdir,int(rightspeed*255)])
            sleep(0.01)
            bus.write_i2c_block_data(ADDRESS,0x01,[leftdir,int(leftspeed*255),rightdir,int(rightspeed*255)])
            
            

    def shutdown(self):
        rospy.loginfo("Shutting down Transbot Driver...")
        bus.write_i2c_block_data(ADDRESS, 0x01, [1, 0, 1, 0])
        rospy.sleep(1)
        rospy.loginfo("Motors stopped.")

    def Buzzercallback(self, request):
        # 蜂鸣器控制，服务端回调函数
        # Buzzer control, server callback function
        if not isinstance(request, BuzzerRequest): return
        bus.write_byte_data(ADDRESS,0x06,request.buzzer)
        sleep(0.01)
        bus.write_byte_data(ADDRESS,0x06,request.buzzer)
        response = BuzzerResponse()
        response.result = True
        return response
        
    def Servocallback(self, request):
        # 舵机控制，服务端回调函数
        # SERVO control, server callback function
        if not isinstance(request, ServoRequest): return
        bus.write_i2c_block_data(ADDRESS,3,[request.servoid,request.angle])
        sleep(0.01)
        bus.write_i2c_block_data(ADDRESS,3,[request.servoid,request.angle])
        response = ServoResponse()
        response.result = True
        return response
        
    def LEDBLUEcallback(self, request):
        # 蓝色LED控制，服务端回调函数
        # Blue LED control, server callback function
        if not isinstance(request, LEDBLUERequest): return
        if request.ledblue == 1:
            GPIO.output(Led_B_pin, GPIO.LOW)
        elif request.ledblue == 0:
            GPIO.output(Led_B_pin, GPIO.HIGH)
        response = LEDBLUEResponse()
        response.result = True
        return response

    def LEDGREEcallback(self, request):
        # 绿色LED控制，服务端回调函数
        # Green LED control, server callback function
        if not isinstance(request, LEDGREERequest): return
        if request.ledgree == 1:
            GPIO.output(Led_G_pin, GPIO.LOW)
        elif request.ledgree == 0:
            GPIO.output(Led_G_pin, GPIO.HIGH)
        response = LEDGREEResponse()
        response.result = True
        return response

    def Motorcallback(self, request):
        # 电机控制，服务端回调函数
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


if __name__ == '__main__':
    rospy.init_node("driver_node", anonymous=False)
    try:
        driver = transbot_driver()
        # driver.pub_data()
#         driver.pub_qrdata()
        rospy.spin()
    except Exception as e:
        
        rospy.loginfo(e)
        rospy.loginfo("Final!!!")


def gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=30,
    flip_method=0):
    return(
        "nvarguscamerasrc !"
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        %(
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height
        )
    )

class CameraPublisher:
    def __init__(self):
        rospy.init_node("camera_publisher", anonymous=False)
        self.image_pub = rospy.Publisher("/cam", Image, queue_size=10)
        self.bridge = CvBridge()
        self.capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        
        if not self.capture.isOpened():
            rospy.logerr("Camera not found!")
            return
        
    def start_stream(self):
        rospy.loginfo("Starting camera stream...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            start_time = time.time()
            ret, frame = self.capture.read()
            if not ret:
                rospy.logwarn("Failed to capture image")
                continue
                
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(ros_image)
            
            end_time = time.time()
            fps = 1.0 / (end_time - start_time)
            rospy.loginfo(f"Publishing image - FPS: {fps:.2f}")
            
            rate.sleep()
        self.capture.release()
        

if __name__ == "__main__":
    camera = CameraPublisher()
    try:
        camera.start_stream()
    except rospy.ROSInterruptException:
        pass
