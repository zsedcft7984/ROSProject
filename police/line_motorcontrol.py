#!/usr/bin/env python3
# encoding: utf-8
import rospy
import smbus
import cv2 as cv
import numpy as np
from time import sleep
from jetbotmini_msgs.srv import *
from jetbotmini_msgs.msg import *
import threading

# I2C 버스 초기화 (Raspberry Pi에서 I2C 통신을 위한 설정)
bus = smbus.SMBus(1)

# I2C 장치 주소 (0x1B는 모터 제어 장치의 고유 주소)
ADDRESS = 0x1B

# 카메라 캡처를 위한 GStreamer 파이프라인 설정
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

class RosDriver:
    def __init__(self):
        # ROS 서비스 및 퍼블리셔 초기화
        self.srv_set_auto = rospy.Service("/SetAuto", SetAuto, self.SetAuto)  # Add service
        self.srv_Buzzer = rospy.Service("/Buzzer", Buzzer, self.Buzzercallback)
        self.motorcontrol = rospy.Service("/MotorControl", MotorControl, self.Motorcontrol)
        self.volPublisher = rospy.Publisher("/voltage", Battery, queue_size=10)
        # 로봇 동작 관련 속성 (속도, 회전)
        self.SPEED_VALUE = 0.4  # 상수로 정의된 속도
        self.TURN_VALUE = 0.2   # 상수로 정의된 회전값
        # 라인 트레이싱 관련 HSV 색상 범위
        self.Hmin = 100
        self.Smin = 43
        self.Vmin = 46
        self.Hmax = 124
        self.Smax = 255
        self.Vmax = 255
        #모터 초기값: 정지
        self.is_motor_stopped = False
        # 자동 라인 추적 활성화 여부
        self.set_auto = False  # 초기값: 0 (자동 조작 비활성화)
        # 배터리 전압을 주기적으로 퍼블리시하기 위한 Timer 설정
        self.timer = rospy.Timer(rospy.Duration(30), self.battery_callback)
        # 카메라 관련 속성
        self.capture = cv.VideoCapture(gstreamer_pipeline(flip_method=0), cv.CAP_GSTREAMER)
        if not self.capture.isOpened():
            rospy.logerr("Failed to open camera.")
        else:
            self.start_camera_stream()

    # 배터리 전압을 퍼블리시하는 함수
    def battery_callback(self, event):
        AD_value = bus.read_i2c_block_data(ADDRESS, 0x00, 2)
        voltage = ((AD_value[0] << 8) + AD_value[1]) * 13.3 / 1023.0
        battery = Battery()
        battery.Voltage = voltage
        self.volPublisher.publish(battery)
        # if voltage >= 12.6:
        #     rospy.loginfo("Battery voltage is at full charge: %.2f V", voltage)
        # elif voltage >= 10.0:
        #     rospy.loginfo("Battery voltage is normal: %.2f V", voltage)
        # elif voltage >= 9.6:
        #     rospy.logwarn("Battery voltage is low: %.2f V", voltage)
        # else:
        #     rospy.logerr("Battery voltage is critically low: %.2f V", voltage)
        
    # 자동주행 ON/OFF
    def SetAuto(self, request):
        if request.start:
            # 라인트레이싱 시작 전, 현재 모터가 구동 중이면 멈추기
            if not self.set_auto:
                self.stop_motors()
            self.set_auto = True
            rospy.loginfo("set_auto set to 1 (라인트레이싱 활성화)")
        else:
            # 라인트레이싱 모드 종료 시, 모터를 멈추고 수동 모드로 전환
            self.set_auto = False
            self.stop_motors()
            rospy.loginfo("set_auto set to 0 (라인트레이싱 비활성화)")
        return True

    def cancel(self):
        self.srv_Buzzer.shutdown()
        self.volPublisher.unregister()
        rospy.loginfo("Closing the robot...")
        rospy.sleep(1)
    
    #모터 정지
    def stop_motors(self):
        # 이미 멈춘 상태일 때는 stop_motors를 호출하지 않음
        if self.is_motor_stopped:
            return
        bus.write_i2c_block_data(ADDRESS, 0x01, [1, 0, 1, 0])
        rospy.loginfo("Motors stopped.")
        self.is_motor_stopped = True  # 모터가 멈췄다고 상태 업데이트

    #부져 서비스하는 함수    
    def Buzzercallback(self, request):
        if not isinstance(request, BuzzerRequest): return
        bus.write_byte_data(ADDRESS, 0x06, request.buzzer)
        sleep(0.01)
        bus.write_byte_data(ADDRESS, 0x06, request.buzzer)
        return BuzzerResponse(result=True)
    
    #수동 모터 조작 서비스하는 함수
    def Motorcontrol(self, request):
        if self.set_auto:
            rospy.loginfo("Automatic mode is active. Ignoring motor control request.")
            return False
        # 수동 조작시 초기값
        speed = 0.4
        leftdir, rightdir = 1, 1
        # Command에 따른 모터 조작
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

        try:
            bus.write_i2c_block_data(ADDRESS, 0x01, [leftdir, int(speed * 255), rightdir, int(speed * 255)])
            sleep(0.01)
            bus.write_i2c_block_data(ADDRESS, 0x01, [leftdir, int(speed * 255), rightdir, int(speed * 255)])
            rospy.loginfo("Motor control successful.")
            return True
        
        except Exception as e:
            rospy.logerr("Motor control failed: %s", e)
            return MotorcontrolResponse()

    #라인트레이싱 함수
    def colorDisplay(self, image):
        # 자동 모드일 때만 라인 트레이싱 수행
        if not self.set_auto:
            return image, None  # 수동 모드에서는 원본 이미지를 반환하고 종료
        # 이미지 크기 가져오기
        height, width, _ = image.shape
        lower_half_image = image[height // 2:, :].copy()  # 성능을 위해 하단 절반 이미지 복사
        # HSV 변환을 위한 범위 설정
        color_lower = np.array([self.Hmin, self.Smin, self.Vmin], dtype=np.uint8)
        color_upper = np.array([self.Hmax, self.Smax, self.Vmax], dtype=np.uint8)
        # 블러 처리 (lower_half_image만 적용)
        lower_half_image = cv.GaussianBlur(lower_half_image, (5, 5), 0)
        # HSV 변환
        hsv = cv.cvtColor(lower_half_image, cv.COLOR_BGR2HSV)
        hsv = hsv.astype(np.uint8)
        # 색상 범위 내 마스크 생성
        mask = cv.inRange(hsv, color_lower, color_upper)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)
        mask = cv.GaussianBlur(mask, (3, 3), 0)
        # 윤곽선 찾기
        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]

        if len(cnts) > 0:
            self.is_motor_stopped = False
            cnt = max(cnts, key=cv.contourArea)
            (color_x, color_y), color_radius = cv.minEnclosingCircle(cnt)
            if color_radius > 30:  # 일정 크기 이상의 원만 고려
                # 색상이 검출된 영역을 원본 `image` 좌표에 맞게 변환
                color_x = int(color_x)
                color_y = int(color_y + height // 2)  # 하단 절반에서 찾았으므로 원래 위치로 보정
                # 원본 이미지에 원을 그림
                cv.circle(image, (color_x, color_y), int(color_radius), (255, 0, 255), 2)
                # 중심 기준 오차 계산 (width를 활용)
                center_offset = (width // 2 - color_x) / (width // 2)
                left_speed = int((self.SPEED_VALUE - self.TURN_VALUE * center_offset) * 255)
                right_speed = int((self.SPEED_VALUE + self.TURN_VALUE * center_offset) * 255)
                # 속도 제한 (0~255 범위)
                left_speed = max(0, min(255, left_speed))
                right_speed = max(0, min(255, right_speed))
                # 모터 제어 신호 전송
                try:
                    bus.write_i2c_block_data(ADDRESS, 0x01, [1, left_speed, 1, right_speed])  # 전진 모션
                except Exception as e:
                    rospy.logerr(f"Failed to send motor command: {e}")
            else:
                self.stop_motors()
        else:
            self.stop_motors()

        return image, mask
        
    # Ros에서 영상 결과 확인할때
    def start_camera_stream(self):
        rate = rospy.Rate(30)  # Process at 30Hz
        while not rospy.is_shutdown():
            ret, frame = self.capture.read()
            if not ret:
                rospy.logwarn("Failed to read frame from camera.")
                continue
            processed_frame, mask = self.colorDisplay(frame)

            # # Show processed frame using OpenCV
            # cv.imshow("Processed Frame", processed_frame)

            # # Check for key press
            # action = cv.waitKey(1) & 0xFF

            # # If 'q' is pressed, stop the robot and close the window
            # if action == ord('q'):
            #     self.stop_motors()  # Stop motors on exit
            #     break
            
            # # If 's' is pressed, toggle line tracing mode
            # if action == ord('s'):
            #     self.set_auto = not self.set_auto  # Toggle line tracing mode
            #     if self.set_auto:
            #         rospy.loginfo("Line tracing enabled.")
            #     else:
            #         rospy.loginfo("Line tracing disabled.")
            
            # # If 'p' is pressed, return to manual control
            # if action == ord('p'):
            #     self.set_auto = False  # Disable automatic line tracing
            #     self.stop_motors()  # Stop motors on exit

            #     rospy.loginfo("Manual control enabled. Line tracing disabled.")
            
            rate.sleep()
if __name__ == '__main__':
    rospy.init_node("driver_node", anonymous=False)
    try:
        driver = RosDriver()
        rospy.spin()
    except Exception as e:
        rospy.logerr("Error: %s", e)
