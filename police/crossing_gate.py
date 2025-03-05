import cv2
import numpy as np
import time
import serial
from ultralytics import YOLO

# YOLO 모델 및 클래스 이름 설정 (모델 경로와 클래스명은 상황에 맞게 수정)
model = YOLO("./nano5.pt")
class_names = ['bunchiko', 'dragonite', 'garados', 'rosa', 'tree']

# 시리얼 포트 설정 (포트 이름과 속도는 아두이노와 일치해야 합니다)
# Windows: 'COM3' 또는 'COM4', Linux: '/dev/ttyUSB0' 등으로 설정
arduino_port = 'LOLIN(WeMos)D1 R1'  # 자신의 환경에 맞게 수정
baud_rate = 9600
ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)  # 시리얼 연결 안정화 대기

def detection_loop():
    cap = cv2.VideoCapture(0)  # 카메라 사용 (웹캠)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽어올 수 없습니다.")
            break

        # YOLO 모델로 객체 감지
        results = model(frame)
        detection_happened = False

        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls = int(box.cls)
                conf = float(box.conf)
                if conf > 0.7:  # 신뢰도 임계치 (0.7 이상)
                    label = f"{class_names[cls]} ({conf * 100:.1f}%)"
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    detection_happened = True

                    # 특정 객체가 검출되면 아두이노에 "RAISE" 명령 전송
                    if conf > 0.9:
                        print("객체 검출됨. 차단바 올리기 명령 전송.")
                        ser.write(b'RAISE\n')
                        # 추가 동작이나 중복 전송 방지를 위해 잠시 대기 (필요 시 조정)
                        time.sleep(1)

        cv2.imshow("YOLO Object Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    detection_loop()
    ser.close()
