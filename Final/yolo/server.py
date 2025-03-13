from flask import Flask, render_template_string
import cv2
import requests
import time

app = Flask(__name__)

# 아두이노 웹서버 IP 주소 및 명령 URL (환경에 맞게 수정)
arduino_ip = "192.168.137.112"
raise_url = f"http://{arduino_ip}/gate_control?cmd=raise"

def capture_and_open():
    # 웹캠으로부터 이미지 캡처
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    cap.release()
    if ret:
        # 원한다면 이미지 저장: cv2.imwrite("captured.jpg", frame)
        # 아두이노에 문 열라는 명령 전송 (timeout 10초)
        try:
            response = requests.get(raise_url, timeout=10)
            return f"Command executed. Response: {response.text}"
        except Exception as e:
            return f"Error sending command: {e}"
    else:
        return "Failed to capture image from webcam."

# 기본 페이지: 버튼 클릭 시 /capture_open 엔드포인트 호출
@app.route('/')
def index():
    html = """
    <!DOCTYPE html>
    <html lang="ko">
    <head>
        <meta charset="UTF-8">
        <title>Door Control</title>
    </head>
    <body>
        <h1>Door Control Panel</h1>
        <button onclick="window.location.href='/capture_open'">Capture & Open Door</button>
    </body>
    </html>
    """
    return render_template_string(html)

# 버튼 클릭 시 호출되는 엔드포인트
@app.route('/capture_open')
def capture_open():
    result = capture_and_open()
    return f"<h2>{result}</h2><br><a href='/'>Go Back</a>"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
