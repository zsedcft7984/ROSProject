#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import Joy
from jetbotmini_msgs.srv import Motor, MotorRequest
from actionlib_msgs.msg import GoalID
from jetbot_ros.cfg import LineDetectPIDConfig
from dynamic_reconfigure.server import Server

# 전역 변수 설정
control_mode = 'auto'  # 기본값: 자동 주행
mot_start = 0
show = 0

# ROS 노드 초기화
rospy.init_node('robot_control')

# 카메라 스트리밍 설정
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

# 모터 제어 함수
def motor_control(leftspeed, rightspeed):
    motor_client = rospy.ServiceProxy("/Motor", Motor)
    motor_client.wait_for_service()
    request = MotorRequest()
    request.leftspeed = leftspeed
    request.rightspeed = rightspeed
    try:
        response = motor_client.call(request)
        return response.result
    except Exception:
        rospy.loginfo("Motor error")
    return False

# 자동 주행 (라인 트레이싱)
def color_display(image):
    global mot_start
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    color_lower = np.array([156, 43, 46], dtype=np.uint8)
    color_upper = np.array([180, 255, 255], dtype=np.uint8)
    mask = cv.inRange(hsv, color_lower, color_upper)
    cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) > 0:
        cnt = max(cnts, key=cv.contourArea)
        (color_x, color_y), color_radius = cv.minEnclosingCircle(cnt)
        if color_radius > 30:
            cv.circle(image, (int(color_x), int(color_y)), int(color_radius), (255, 0, 255), 2)
            center_x = (320 - color_x) / 320
            if mot_start == 1:
                motor_control(0.5 - 0.4 * center_x, 0.5 + 0.4 * center_x)
    else:
        motor_control(0, 0)
    return image

# 수동 조작 (조이스틱 입력 처리)
def joystick_control(joy_msg):
    speed = joy_msg.axes[1]  # 앞/뒤 이동
    turn = joy_msg.axes[0]   # 좌/우 회전
    motor_control(speed, turn)

# 메인 실행 함수
def main():
    global control_mode, show
    capture = cv.VideoCapture(gstreamer_pipeline(), cv.CAP_GSTREAMER)
    
    if not capture.isOpened():
        rospy.loginfo("카메라를 열 수 없습니다.")
        return
    
    # 조이스틱 입력 구독
    joy_subscriber = rospy.Subscriber("/joy", Joy, joystick_control)

    while not rospy.is_shutdown():
        ret, frame = capture.read()
        if not ret:
            rospy.loginfo("프레임을 읽을 수 없습니다.")
            break
        
        if control_mode == 'auto':
            frame = color_display(frame)  # 자동 주행 실행
        elif control_mode == 'manual':
            pass  # 조이스틱 입력으로 모터가 조작됨

        # 화면 출력
        if show == 1:
            cv.imshow('Frame', frame)
        
        action = cv.waitKey(10) & 0xFF
        if action == ord('q'):
            break
        elif action == ord('m'):  # 'm' 키 입력 시 모드 전환
            control_mode = 'manual' if control_mode == 'auto' else 'auto'
            rospy.loginfo(f"모드 변경: {control_mode}")
    
    capture.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
