import os
import datetime
import cv2
import numpy as np
import requests
import time
from ultralytics import YOLO
import roslibpy

# YOLO 모델 및 클래스 이름 설정
model = YOLO("./knife3.pt")  # 모델 파일 경로 설정
class_names = ['knife']  # 감지할 객체 클래스

# Jetbot 스트리밍 URL
url = "http://192.168.137.26:8080/stream?topic=/usb_cam/image_raw"

# ROS 연결 설정
ros = roslibpy.Ros(host='192.168.137.26', port=9090)
ros.run()  # ROS에 연결
print("Connected to ROS")

# 부저 제어 서비스 연결 (예: /buzzer_service)
buzzer_service = roslibpy.Service(ros, '/Buzzer', 'jetbotmini_msgs/Buzzer')  # 서비스 이름과 타입에 맞게 수정

# 마지막 부저 호출 시각 (전역 변수)
last_buzzer_time = 0

# 프레임 처리 간격 (예: 60 FPS에서 20번째 프레임마다 객체 감지 -> 약 0.3초 간격)
frame_interval = 20

def save_image(frame, label, order):
    """이미지를 라벨, 순서, 저장된 시간으로 파일로 저장하는 함수"""
<<<<<<< HEAD
    current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M")  # 날짜 및 시간 형식 조정
=======
    current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")  # 날짜 및 시간 형식 조정
>>>>>>> 29b853930d0c3cc726cf77d34c9e1c71a41bb810
    save_dir = "saved_images"

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

<<<<<<< HEAD
    # 파일명: "knife_1_2025-03-06_15:30.jpg"
=======
    # 파일명: "knife_1_2025-03-06_15:30:00.jpg"
>>>>>>> 29b853930d0c3cc726cf77d34c9e1c71a41bb810
    filename = f"{save_dir}/{label}_{order}_{current_time}.jpg"
    cv2.imwrite(filename, frame)
    print(f"Image saved: {filename}")


def call_buzzer_service(state):
    """부저 제어 서비스 호출 (1: ON, 0: OFF)"""
    request = roslibpy.Message({'buzzer': state})
    response = buzzer_service.call(request)
    if response:
        print(f"Buzzer {'ON' if state == 1 else 'OFF'}")
    else:
        print("Failed to call buzzer service")

def detection_loop():
    """HTTP 스트리밍으로 프레임을 읽어 객체 감지 후 화면에 표시"""
    global last_buzzer_time
    stream = requests.get(url, stream=True)
    bytes_data = b""
    order = 1  # 이미지 저장 순서 추적용 변수
    frame_count = 0  # 프레임 카운터

    for chunk in stream.iter_content(chunk_size=1024):
        bytes_data += chunk
        a = bytes_data.find(b'\xff\xd8')  # JPEG 시작
        b = bytes_data.find(b'\xff\xd9')  # JPEG 끝

        if a != -1 and b != -1:
            jpg = bytes_data[a:b+2]  # JPEG 이미지 추출
            bytes_data = bytes_data[b+2:]  # 버퍼에서 제거

            # OpenCV로 이미지 디코딩
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

            frame_count += 1
            if frame_count % frame_interval == 0:  # 지정된 프레임마다 객체 감지
                results = model(frame)

                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        cls = int(box.cls)
                        conf = float(box.conf)
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        if conf > 0.8:  # 신뢰도 0.8 이상인 경우만 처리
                            label = f"{class_names[cls]} ({conf * 100:.1f}%)"
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(frame, label, (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                            # 신뢰도가 0.9 이상이고, 클래스가 'knife'인 경우 부저 제어 (중복 작동 방지)
                            if class_names[cls] == "knife":
                                current_time = time.time()
                                if (current_time - last_buzzer_time) > 15:
                                    print(f"Knife detected with high confidence! Order: {order}")
                                    save_image(frame, class_names[cls], order)
                                    order += 1
                                    last_buzzer_time = current_time

                                    # 부저 ON
                                    call_buzzer_service(1)
                                    # 5초 후 부저 OFF
                                    time.sleep(5)
                                    call_buzzer_service(0)

                # 결과 프레임 화면에 표시
                cv2.imshow("YOLO Object Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    stream.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    detection_loop()
    ros.close()
