import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from jetbotmini import Camera, Robot
import cv2
import numpy as np
from matplotlib import pyplot as plt
from IPython import display

# 카메라 및 로봇 객체
camera = Camera.instance(width=300, height=300)
robot = Robot()

# PID 회전 제어 객체
turn_gain_pid = IncrementalPID(0.15, 0, 0.05)

# 차량 제어 명령을 처리하는 함수
def command_callback(msg):
    command = msg.data
    if command == "forward":
        robot.forward(0.5)  # 50% 속도로 전진
    elif command == "backward":
        robot.backward(0.5)  # 50% 속도로 후진
    elif command == "left":
        robot.left(0.5)  # 50% 속도로 좌회전
    elif command == "right":
        robot.right(0.5)  # 50% 속도로 우회전
    elif command == "stop":
        robot.stop()  # 차량 정지

# PID 값을 발행하는 함수
def publish_pid_values():
    target_value_speed = 0
    target_value_turn_gain = 0
    
    frame = camera.value
    frame = cv2.resize(frame, (300, 300))
    frame_ = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    color_lower = np.array([156, 43, 46])
    color_upper = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, color_lower, color_upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    mask = cv2.GaussianBlur(mask, (3, 3), 0)

    # 컨투어 찾기
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    if len(cnts) > 0:
        cnt = max(cnts, key=cv2.contourArea)
        (color_x, color_y), color_radius = cv2.minEnclosingCircle(cnt)
        if color_radius > 10:
            # 색상 인식된 경우 PID 계산
            center = (150 - color_x) / 150
            turn_gain_pid.SetStepSignal(center)
            turn_gain_pid.SetInertiaTime(0.2, 0.1)
            
            target_value_turn_gain = 0.15 + abs(turn_gain_pid.SystemOutput)
            if target_value_turn_gain < 0:
                target_value_turn_gain = 0
            elif target_value_turn_gain > 2:
                target_value_turn_gain = 2

            # PID를 통해 회전 값 설정
            target_value_speedl = 0.4 - target_value_turn_gain * center
            target_value_speedr = 0.4 + target_value_turn_gain * center
            
            # 로봇 제어
            if target_value_speedl < 0.3:
                target_value_speedl = 0
            elif target_value_speedl > 1:
                target_value_speedl = 1
            if target_value_speedr < 0.3:
                target_value_speedr = 0
            elif target_value_speedr > 1:
                target_value_speedr = 1
                
            robot.set_motors(target_value_speedl, target_value_speedr)

    else:
        # 타겟 없으면 로봇 멈춤
        robot.stop()
    
    return target_value_speedl, target_value_speedr

# 카메라 값 구독 및 제어
def camera_callback(msg):
    # 카메라 이미지 처리 및 PID 값 발행
    speedl, speedr = publish_pid_values()

    # PID 값을 토픽으로 발행
    pid_pub_left.publish(speedl)
    pid_pub_right.publish(speedr)

    # 이미지 값을 JPEG 형식으로 발행
    image_pub.publish(msg)

# ROS 노드 초기화
rospy.init_node('robot_control_node')

# 차량 제어 명령을 받기 위한 구독자 생성
command_sub = rospy.Subscriber('/robot/command/move', String, command_callback)

# PID 값 발행용 토픽 생성
pid_pub_left = rospy.Publisher('/robot/pid_left', Float32, queue_size=10)
pid_pub_right = rospy.Publisher('/robot/pid_right', Float32, queue_size=10)

# 카메라 이미지 발행용 토픽 생성
image_pub = rospy.Publisher('/robot/camera/image', Image, queue_size=10)

# 카메라 값 구독 (카메라 이미지를 구독하여 PID 제어 값 계산 후 발행)
camera.observe(camera_callback, names='value')

# ROS 루프 실행
rospy.spin()
