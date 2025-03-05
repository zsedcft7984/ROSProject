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
        self.volPublisher = rospy.Publisher("/voltage", Battery, queue_size=10)

    def cancel(self):
        self.srv_Buzzer.shutdown()
        self.srv_Servo.shutdown()
        self.srv_LEDBLUE.shutdown()
        self.srv_LEDGREE.shutdown()
        self.srv_Motor.shutdown()
        self.volPublisher.unregister()
        GPIO.cleanup()
        rospy.loginfo("Close the robot...")
        rospy.sleep(1)

    def pub_data(self):
        # 发布小车电池电压
        # Release trolley battery voltage
        while not rospy.is_shutdown():
            sleep(30)
            AD_value = bus.read_i2c_block_data(ADDRESS,0x00,2)
            voltage = ((AD_value[0] << 8) + AD_value[1]) * 13.3 / 1023.0
            battery = Battery()
            battery.Voltage = voltage
            self.volPublisher.publish(battery)

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
        driver.pub_data()
        rospy.spin()
    except:
        rospy.loginfo("Final!!!")





