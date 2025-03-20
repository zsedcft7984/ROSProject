import cv2
import numpy as np
import time
import requests
from ultralytics import YOLO

# YOLO 모델 및 클래스 이름 설정
model = YOLO("./people.pt")
class_names = ['bunchiko', 'dragonite', 'garados', 'rosa', 'tree']

# 아두이노 웹서버 IP 주소 (아두이노가 Wi‑Fi 네트워크에 연결된 IP)
arduino_ip = "192.168.137.134"  # 자신의 환경에 맞게 수정하세요.
raise_url = f"http://{arduino_ip}/gate_control?cmd=raise"
lower_url = f"http://{arduino_ip}/gate_control?cmd=lower"

# 디바운스(중복 전송 방지)를 위한 타이머 변수

last_command_time = 0
command_cooldown = 5  # 5초 동안 중복 전송 방지

# 모델 실행 주기를 10초로 설정
frame_interval = 10  # 10초마다 한 번만 실행
last_frame_time = time.time()  # 마지막 모델 실행 시간 기록

def detection_loop():
    global last_command_time, last_frame_time
    cap = cv2.VideoCapture(0)  # 웹캠 사용
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽어올 수 없습니다.")
            break

        current_time = time.time()
        if current_time - last_frame_time >= frame_interval:
            # YOLO 모델로 객체 감지
            results = model(frame)
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    cls = int(box.cls)
                    conf = float(box.conf)
                    if conf > 0.7:
                        label = f"{class_names[cls]} ({conf * 100:.1f}%)"
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, label, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                        # 신뢰도가 0.9 이상이면, 일정 시간 이후에만 무선 명령 전송
                        if conf > 0.9 and (time.time() - last_command_time) > command_cooldown:
                            print("객체 검출됨")
                            try:
                                response = requests.get(raise_url, timeout=5)
                                print("차단바 올리기 명령 전송. 응답:", response.text)
                            except Exception as e:
                                print("명령 전송 에러:", e)
                            last_command_time = time.time()
            last_frame_time = current_time

        cv2.imshow("YOLO Object Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    detection_loop()
