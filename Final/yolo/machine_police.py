import os
import datetime
import cv2
import numpy as np
import requests
import time
import boto3
from botocore.client import Config
from dotenv import dotenv_values
from ultralytics import YOLO
import roslibpy

# YOLO 모델 및 클래스 설정
model = YOLO("Final\yolo\knife.pt")
class_names = ['knife']

# Jetbot 스트리밍 URL
url = "http://192.168.137.160:8080/stream?topic=/usb_cam/image_raw"

# ROS 연결 설정
ros = roslibpy.Ros(host='192.168.137.160', port=9090)
ros.run()
print("Connected to ROS")

# ROS 토픽 생성
yolo_topic = roslibpy.Topic(ros, '/yolo_detection_results', 'std_msgs/String')

# 부저 서비스
buzzer_service = roslibpy.Service(ros, '/Buzzer', 'jetbotmini_msgs/Buzzer')

# 부저 호출 제한 시간 (15초)
last_buzzer_time = 0

# 프레임 처리 간격
frame_interval = 20

# AWS S3 설정
config = dotenv_values('Final\yolo\.env')
s3 = boto3.client(
    's3',
    aws_access_key_id=config['AWS_ACCESS_KEY'],
    aws_secret_access_key=config['AWS_SECRET_KEY'],
    config=Config(signature_version='s3v4')
)

def upload_to_s3(filename):
    """이미지를 AWS S3에 업로드하는 함수"""
    try:
        with open(filename, 'rb') as data:
            s3.upload_fileobj(data, config['AWS_BUCKET_NAME'], os.path.basename(filename))
        print(f"Uploaded to S3: {filename}")
    except Exception as e:
        print(f"Failed to upload {filename} to S3: {e}")

def save_image(frame, label, order):
    """객체 감지된 이미지를 저장하는 함수"""
    current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    os.makedirs("saved_images", exist_ok=True)
    filename = f"saved_images/{label}-{order}-{current_time}.jpg"
    cv2.imwrite(filename, frame)
    print(f"Image saved: {filename}")

    # 이미지 저장 후 자동으로 S3에 업로드
    upload_to_s3(filename)

def call_buzzer_service(state):
    """부저 ON/OFF"""
    buzzer_service.call(roslibpy.Message({'buzzer': state}))
    print(f"Buzzer {'ON' if state == 1 else 'OFF'}")

def detection_loop():
    """객체 감지 및 ROS 메시지 전송"""
    global last_buzzer_time
    stream = requests.get(url, stream=True)
    bytes_data = b""
    order = 1
    frame_count = 0

    for chunk in stream.iter_content(chunk_size=1024):
        bytes_data += chunk
        a, b = bytes_data.find(b'\xff\xd8'), bytes_data.find(b'\xff\xd9')

        if a != -1 and b != -1:
            jpg = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

            frame_count += 1
            if frame_count % frame_interval == 0:
                results = model(frame)

                for result in results:
                    for box in result.boxes:
                        cls, conf = int(box.cls), float(box.conf)
                        x1, y1, x2, y2 = map(int, box.xyxy[0])

                        if conf > 0.8:
                            label = f"{class_names[cls]} ({conf * 100:.1f}%)"
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(frame, label, (x1, y1 - 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                            # Knife 감지 시 부저 작동 (15초 간격)
                            if class_names[cls] == "knife" and time.time() - last_buzzer_time > 15:
                                print(f"Knife detected! Order: {order}")
                                save_image(frame, "knife", order)
                                order += 1
                                last_buzzer_time = time.time()
                                call_buzzer_service(1)

                # ROS 메시지 전송
                yolo_topic.publish(roslibpy.Message({'data': label}))

                # 결과 출력
                cv2.imshow("YOLO Object Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    stream.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    detection_loop()
    ros.close()
