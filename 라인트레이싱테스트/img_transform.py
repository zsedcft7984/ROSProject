#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import pyzbar.pyzbar as pyzbar
from PIL import Image as Img, ImageDraw, ImageFont
import numpy as np

def topic(msg):
    if not isinstance(msg, Image):
        return
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    frame = cv.resize(frame, (640, 480))
    
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    barcodes = pyzbar.decode(gray)
    for barcode in barcodes:
        (x, y, w, h) = barcode.rect
        cv.rectangle(frame, (x, y), (x + w, y + h), (225, 0, 0), 5)
        encoding = 'UTF-8'
        barcodeData = barcode.data.decode(encoding)
        barcodeType = barcode.type
        pilimg = Img.fromarray(frame)
        draw = ImageDraw.Draw(pilimg)
        draw.text((x, y - 25), str(barcode.data), fill=(255, 0, 0))
        # frame = cv.cvtColor(np.array(pilimg), cv.COLOR_RGB2BGR)
        frame = np.array(pilimg)
        print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
    
    if img_flip == True: frame = cv.flip(frame, 1)
    # opencv mat ->  ros msg
    msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    pub_img.publish(msg)


if __name__ == '__main__':
    rospy.init_node("pub_img", anonymous=False)
    img_topic = rospy.get_param("~img_topic", "/csi_cam_0/image_raw")
    img_flip = rospy.get_param("~img_flip", False)
    sub_img = rospy.Subscriber(img_topic, Image, topic)
    pub_img = rospy.Publisher("/image", Image, queue_size=10)
    rate = rospy.Rate(2)
    rospy.spin()
