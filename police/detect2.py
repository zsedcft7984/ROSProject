from flask import Flask
from flask_socketio import SocketIO
import threading
import cv2
import numpy as np
import requests
from ultralytics import YOLO
import time

# Flask 및 SocketIO 설정
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

# YOLO 모델 및 클래스 이름 설정
model = YOLO("./nano5.pt")
class_names = ['bunchiko', 'dragonite', 'garados', 'rosa', 'tree']

# Jetbot 스트리밍 URL (스트림 주소를 실제 주소로 수정하세요)
url = "http://192.168.137.53:8080/stream?topic=/usb_cam/image_raw"

def detection_loop():
    """HTTP 스트리밍으로 프레임을 읽어 객체 감지 후 웹소켓 이벤트 전송"""
    stream = requests.get(url, stream=True)
    bytes_data = b""
    for chunk in stream.iter_content(chunk_size=1024):
        bytes_data += chunk
        a = bytes_data.find(b'\xff\xd8')  # JPEG 시작
        b = bytes_data.find(b'\xff\xd9')  # JPEG 끝

        if a != -1 and b != -1:
            jpg = bytes_data[a:b+2]  # JPEG 이미지 추출
            bytes_data = bytes_data[b+2:]  # 버퍼에서 제거

            # OpenCV로 이미지 디코딩
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

            # YOLO 모델로 객체 감지
            results = model(frame)
            detection_happened = False
            detected_objects = []

            # 감지된 객체에 대해 바운딩 박스 그리기
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    cls = int(box.cls)
                    conf = float(box.conf)
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    if conf > 0.7:
                        label = f"{class_names[cls]}: {conf*100:.2f}%"
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, label, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        detection_happened = True
                        detected_objects.append(label)

            # 감지 이벤트가 발생하면 웹소켓으로 데이터 전송
            if detection_happened:
                data = {'objects': detected_objects}
                socketio.emit('object_detected', data)
                print("Detection event sent:", data)
                time.sleep(10)  # 이벤트 스팸 방지

            # 결과 프레임 화면에 표시
            cv2.imshow("Jetbot Object Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    stream.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # 별도의 스레드에서 객체 감지 루프 실행
    detection_thread = threading.Thread(target=detection_loop)
    detection_thread.daemon = True
    detection_thread.start()

    # Flask-SocketIO 서버 실행
    socketio.run(app, port=5000, debug=True)
