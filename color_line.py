#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import cv2 as cv
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import os
import rospy
import threading
from time import sleep
from sensor_msgs.msg import Joy
from jetbotmini_msgs.msg import *
from jetbotmini_msgs.srv import *
from actionlib_msgs.msg import GoalID
from jetbot_ros.cfg import LineDetectPIDConfig
from dynamic_reconfigure.server import Server

global color_lower
global color_upperv
global show
global mot_start


color_lower = np.array([156,43,46], dtype=np.uint8)
color_upper = np.array([180, 255, 255], dtype=np.uint8)
show = 0
mot_start = 0

def gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=30,
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

class JoyTeleop:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.Buzzer_value = False
        self.LEDBLUE_value = False
        self.LEDGREE_value = False
        self.speed_value = 0.5
        self.turn_value = 0.4
        self.angle1_value = 0
        self.angle2_value = 0
        self.Joy_active = False
        self.cancel_time = time.time()
        self.pub_goal = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        self.pub_JoyState = rospy.Publisher("/JoyState", JoyState, queue_size=10)
        self.Motor_client = rospy.ServiceProxy("/Motor", Motor)
        Server(LineDetectPIDConfig, self.dynamic_reconfigure_callback)

    def cancel(self):
        self.pub_goal.unregister()
        self.pub_JoyState.unregister()
        self.Motor_client.close()
        
    def colorDisplay(self, image, font_path):
        color_lower = np.array([self.Hmin, self.Smin, self.Vmin], dtype=np.uint8)
        color_upper = np.array([self.Hmax, self.Smax, self.Vmax], dtype=np.uint8)
        image=cv.GaussianBlur(image,(5,5),0)     
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        hsv.astype(np.uint8)
        mask=cv.inRange(hsv,color_lower,color_upper)  
        mask=cv.erode(mask,None,iterations=2)
        mask=cv.dilate(mask,None,iterations=2)
        mask=cv.GaussianBlur(mask,(3,3),0)     
        cnts=cv.findContours(mask.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts)>0:
            cnt = max (cnts,key=cv.contourArea)
            (color_x,color_y),color_radius=cv.minEnclosingCircle(cnt)
            if color_radius > 30:
                # 将检测到的颜色标记出来
                # Mark the detected color
                cv.circle(image,(int(color_x),int(color_y)),int(color_radius),(255,0,255),2)   
                center_x = (320 - color_x)/320
                if mot_start == 1:
                    self.Motor_srv(
                        float(self.speed_value - self.turn_value * center_x),
                        float(self.speed_value + self.turn_value * center_x)
                    )
        else:
            self.Motor_srv(0,0)
        return image, mask

    def cancel_nav(self):
        # 发布move_base取消命令
        # Publish move_ Base cancel command
        now_time = time.time()
        if now_time - self.cancel_time > 1:
            self.Joy_active = not self.Joy_active
            self.pub_JoyState.publish(JoyState(self.Joy_active))
            self.pub_goal.publish(GoalID())
            self.cancel_time=now_time

    def Motor_srv(self, leftspeed, rightspeed):
        '''
        电机控制
        Motor control
        :param value:
         [  leftspeed：   0,1
            rightspeed：  0,1]
        '''
        self.Motor_client.wait_for_service()
        request = MotorRequest()
        request.leftspeed = leftspeed
        request.rightspeed = rightspeed
        try:
            response = self.Motor_client.call(request)
            if isinstance(response, MotorResponse): return response.result
        except Exception:
            rospy.loginfo("Motor error")
        return False
        
    def dynamic_reconfigure_callback(self, config, level):
        print ("dynamic_reconfigure_callback!!!")
        self.speed_value = config['speed']
        self.turn_value = config['turn']
        
        self.Hmin = config['Hmin']
        self.Smin = config['Smin']
        self.Vmin = config['Vmin']
        self.Hmax = config['Hmax']
        self.Smax = config['Smax']
        self.Vmax = config['Vmax']
        
        return config        

if __name__ == '__main__':
    rospy.init_node('color_line')
    joy = JoyTeleop()
    font_path = "../font/Block_Simplified.TTF"
    capture = cv.VideoCapture(gstreamer_pipeline(flip_method=0), cv.CAP_GSTREAMER)
    cv_edition = cv.__version__
    if cv_edition[0] == '3': capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
    else: capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    while capture.isOpened():
        start = time.time()
        ret, frame = capture.read()
        action = cv.waitKey(10) & 0xFF
        frame, binary = joy.colorDisplay(frame, font_path)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        cv.imshow('frame', frame)
        if action == ord('q') or action == 113: break
        elif action == ord('i') or action == 105: show = 1
        elif action == 32: show = 0
        elif action == ord('s'): mot_start = 1
        if show == 1:
            cv.imshow('frame', binary)
        elif show == 0:
            cv.imshow('frame', frame)
    joy.Motor_srv(0,0)
    capture.release()
    cv.destroyAllWindows()

