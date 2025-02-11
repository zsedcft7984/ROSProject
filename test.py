#!/usr/bin/env python3
# encoding: utf-8
import rospy
import random
import threading
from time import sleep
from jetbotmini_msgs.msg import *
from jetbotmini_msgs.srv import *
import RPi.GPIO as GPIO
import smbus
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# I2C 설정
bus = smbus.SMBus(1)
ADDRESS = 0x1B

# LED 및 버튼 핀 번호 설정
Led_G_pin = 24
Led_B_pin = 23
Key1_pin = 8

GPIO.setmode(GPIO.BCM)
GPIO.setup(Led_G_pin, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(Led_B_pin, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(Key1_pin, GPIO.IN)

print("로봇 시작!")

class JetBotDriver:
    def __init__(self):
        # 종료 시 실행할 함수 등록
        rospy.on_shutdown(self.shutdown)

        # ROS 서비스 및 퍼블리셔 설정
        self.srv_Buzzer = rospy.Service("/Buzzer", Buzzer, self.buzzer_callback)
        self.srv_LED_BLUE = rospy.Service("/LEDBLUE", LEDBLUE, self.led_blue_callback)
        self.srv_LED_GREEN = rospy.Service("/LEDGREE", LEDGREE, self.led_green_callback)
        self.srv_Motor = rospy.Service("/Motor", Motor, self.motor_callback)
        self.volPublisher = rospy.Publisher("/voltage", Battery, queue_size=10)

        # 카메라 이미지 퍼블리셔 설정
        self.imagePublisher = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
        self.bridge = CvBridge()

        # 카메라 설정
        self.capture = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if not self.capture.isOpened():
            rospy.logerr("카메라를 열 수 없습니다.")
            exit()

    def gstreamer_pipeline(
        self,
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

    def publish_camera_data(self):
        """카메라 영상을 주기적으로 퍼블리시하는 함수"""
        while not rospy.is_shutdown():
            ret, frame = self.capture.read()
            if ret:
                try:
                    # OpenCV 이미지를 ROS 메시지로 변환
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    self.imagePublisher.publish(ros_image)
                except Exception as e:
                    rospy.logerr("이미지 퍼블리시 오류: %s" % e)
            else:
                rospy.logwarn("카메라에서 영상을 읽을 수 없습니다.")

    def shutdown(self):
        """로봇 종료 시 실행되는 함수"""
        self.srv_Buzzer.shutdown()
        self.srv_LED_BLUE.shutdown()
        self.srv_LED_GREEN.shutdown()
        self.srv_Motor.shutdown()
        self.volPublisher.unregister()
        self.imagePublisher.unregister()
        self.capture.release()
        GPIO.cleanup()
        rospy.loginfo("로봇 종료...")
        rospy.sleep(1)

    def publish_battery_data(self):
        """배터리 전압 정보를 주기적으로 발행하는 함수"""
        while not rospy.is_shutdown():
            sleep(30)
            AD_value = bus.read_i2c_block_data(ADDRESS, 0x00, 2)
            voltage = ((AD_value[0] << 8) + AD_value[1]) * 13.3 / 1023.0
            battery = Battery()
            battery.Voltage = voltage
            self.volPublisher.publish(battery)

    def buzzer_callback(self, request):
        """부저(경고음) 제어 함수"""
        if not isinstance(request, BuzzerRequest):
            return
        bus.write_byte_data(ADDRESS, 0x06, request.buzzer)
        sleep(0.01)
        bus.write_byte_data(ADDRESS, 0x06, request.buzzer)
        response = BuzzerResponse()
        response.result = True
        return response

    def led_blue_callback(self, request):
        """파란색 LED 제어 함수"""
        if not isinstance(request, LEDBLUERequest):
            return
        GPIO.output(Led_B_pin, GPIO.LOW if request.ledblue == 1 else GPIO.HIGH)
        response = LEDBLUEResponse()
        response.result = True
        return response

    def led_green_callback(self, request):
        """초록색 LED 제어 함수"""
        if not isinstance(request, LEDGREERequest):
            return
        GPIO.output(Led_G_pin, GPIO.LOW if request.ledgree == 1 else GPIO.HIGH)
        response = LEDGREEResponse()
        response.result = True
        return response

    def motor_callback(self, request):
        """모터 제어 함수"""
        if not isinstance(request, MotorRequest):
            return

        # 왼쪽 모터 방향 및 속도 설정
        leftdir = 1 if request.leftspeed >= 0 else 0
        leftspeed = abs(request.leftspeed)

        # 오른쪽 모터 방향 및 속도 설정
        rightdir = 1 if request.rightspeed >= 0 else 0
        rightspeed = abs(request.rightspeed)

        # I2C를 통해 모터 속도 및 방향 설정
        bus.write_i2c_block_data(
            ADDRESS, 0x01, [leftdir, int(leftspeed * 255), rightdir, int(rightspeed * 255)]
        )
        sleep(0.01)
        bus.write_i2c_block_data(
            ADDRESS, 0x01, [leftdir, int(leftspeed * 255), rightdir, int(rightspeed * 255)]
        )

        response = MotorResponse()
        response.result = True
        return response


if __name__ == "__main__":
    rospy.init_node("driver_node", anonymous=False)
    try:
        driver = JetBotDriver()
        driver.publish_battery_data()
        driver.publish_camera_data()  # 카메라 데이터 퍼블리시 시작
        rospy.spin()
    except:
        rospy.loginfo("오류 발생, 종료합니다.")
