import rospy
import smbus
import cv2 as cv
import numpy as np
from time import sleep
from jetbotmini_msgs.srv import *
from pid import PID  # You can install this or create a simple PID controller class

bus = smbus.SMBus(1)
ADDRESS = 0x1B
# PID controller class (if you don't want to use external library)
class PIDController:
    def __init__(self, p=0.5, i=0.1, d=0.1):
        self.p = p
        self.i = i
        self.d = d
        self.last_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.last_error
        output = self.p * error + self.i * self.integral + self.d * derivative
        self.last_error = error
        return output

class transbot_driver:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.srv_Buzzer = rospy.Service("/Buzzer", Buzzer, self.Buzzercallback)
        self.srv_Motor = rospy.Service("/Motor", Motor, self.Motorcallback)
        self.motorcontrol = rospy.Service("/MotorControl", MotorControl, self.Motorcontrol)

        self.capture = cv.VideoCapture(gstreamer_pipeline(flip_method=0), cv.CAP_GSTREAMER)
        
        if not self.capture.isOpened():
            rospy.logerr("Failed to open camera.")
        else:
            self.start_camera_stream()

        # PID controller for line tracing
        self.pid = PIDController(p=0.5, i=0.1, d=0.1)

    def cancel(self):
        self.srv_Buzzer.shutdown()
        self.srv_Motor.shutdown()
        self.capture.release()
        rospy.loginfo("Closing the robot...")
        rospy.sleep(1)

    def start_camera_stream(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            ret, frame = self.capture.read()
            if not ret:
                continue
            processed_frame, mask = self.colorDisplay(frame)
            rate.sleep()

    def colorDisplay(self, image):
        # Convert the image to HSV
        color_lower = np.array([self.Hmin, self.Smin, self.Vmin], dtype=np.uint8)
        color_upper = np.array([self.Hmax, self.Smax, self.Vmax], dtype=np.uint8)

        # Pre-process image
        image = cv.GaussianBlur(image, (5, 5), 0)
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
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
            
            if color_radius > 30:  # Only consider large enough contours
                cv.circle(image, (int(color_x), int(color_y)), int(color_radius), (255, 0, 255), 2)

                # Calculate error based on the position of the line
                error = 320 - color_x  # Error is the difference from the center (320 is the middle of the frame)
                pid_output = self.pid.compute(error)  # Get PID output for correction
                
                # Adjust motor speeds based on the PID output
                left_speed = self.speed_value - pid_output
                right_speed = self.speed_value + pid_output
                
                self.Motor_srv(left_speed, right_speed)
        else:
            # If no line detected, stop
            self.Motor_srv(0, 0)

        return image, mask

    def Motor_srv(self, left_speed, right_speed):
        # Call the motor service to adjust the motor speeds
        bus.write_i2c_block_data(ADDRESS, 0x01, [1, int(left_speed * 255), 1, int(right_speed * 255)])
        sleep(0.01)

    def Buzzercallback(self, request):
        if not isinstance(request, BuzzerRequest): return
        bus.write_byte_data(ADDRESS, 0x06, request.buzzer)
        sleep(0.01)
        bus.write_byte_data(ADDRESS, 0x06, request.buzzer)
        return BuzzerResponse(result=True)

    def Motorcallback(self, request):
        if not isinstance(request, MotorRequest): return
        leftdir = 1 if request.leftspeed >= 0 else 0
        rightdir = 1 if request.rightspeed >= 0 else 0
        bus.write_i2c_block_data(ADDRESS, 0x01, [leftdir, int(abs(request.leftspeed) * 255),
                                                 rightdir, int(abs(request.rightspeed) * 255)])
        sleep(0.01)
        return MotorResponse(result=True)
    
    def Motorcontrol(self, request):
        print(request)
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

        bus.write_i2c_block_data(ADDRESS, 0x01, [leftdir, int(speed * 255), rightdir, int(speed * 255)])
        sleep(0.01)

if __name__ == '__main__':
    rospy.init_node("driver_node", anonymous=False)
    try:
        driver = transbot_driver()
        rospy.spin()
    except Exception as e:
        rospy.loginfo(e)
        rospy.loginfo("Final!!!")
