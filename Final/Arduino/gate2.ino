#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

const char* ssid = "APT_main";            // Wi-Fi 이름
const char* password = "123456789";       // Wi-Fi 비밀번호

ESP8266WebServer server(80);
Servo gateServo;

#define ULTRASONIC_TRIGGER_PIN D2
#define ULTRASONIC_ECHO_PIN D1
const int distanceThreshold = 10; // 초음파 감지 임계값 (cm)

bool isOperating = false;   // 모터 동작 상태
bool manualCommand = false; // 웹에서 수동 명령("up")이 들어왔는지 여부

// 자동 닫힘을 위한 설정 (예: 5초 동안 감지가 없으면 닫음)
const unsigned long autoCloseDelay = 5000; // 밀리초 단위
unsigned long lastDetectionTime = 0;       // 마지막 감지 시간 기록

// 마지막 메시지를 저장할 전역 변수
String lastMessage = "";

// 함수 선언
void openGate();
void closeGate();
void operateSubMotor();

void setup() {
  Serial.begin(115200);
  gateServo.attach(D5, 700, 2800); // 서보모터 핀 및 펄스 설정
  
  pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  WiFi.begin(ssid, password);
  Serial.print("Wi-Fi 연결 중...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("연결됨, IP 주소: ");
  Serial.println(WiFi.localIP().toString());

  // 웹 서버 엔드포인트 설정 (원격 명령 처리)
  server.on("/gate_control", HTTP_GET, [](){
    String command = server.arg("cmd");
    command.trim();
    // "up" 명령: 수동 모드, 문은 계속 열림 (자동 닫힘 없음)
    if (command.equalsIgnoreCase("up")) {
      if (!isOperating) {
        isOperating = true;
        manualCommand = true;  // 수동 명령 설정
        openGate();
        operateSubMotor();
        lastMessage = "OpenService";  // 메시지 업데이트
        server.send(200, "text/plain", "Gate Raised (Manual: Stays Open)");
      }
    } 
    // "raise" 명령: 자동 모드, 5초 후 자동 닫힘
    else if (command.equalsIgnoreCase("raise")) {
      if (!isOperating) {
        isOperating = true;
        manualCommand = false; // 자동 모드
        openGate();
        operateSubMotor();
        lastMessage = "OpenService";  // 메시지 업데이트
        server.send(200, "text/plain", "Gate Raised (Auto: Will close in 5 sec)");
        lastDetectionTime = millis();  // 타이머 시작
      }
    } 
    else if (command.equalsIgnoreCase("lower")) {
      manualCommand = false;
      closeGate();
      isOperating = false;
      lastMessage = "Gate Closed";
      server.send(200, "text/plain", "Gate Closed");
    } 
    else {
      server.send(400, "text/plain", "Invalid Command");
    }
  });

  // /get_message 엔드포인트: 마지막 메시지를 반환
  server.on("/get_message", HTTP_GET, [](){
    server.send(200, "text/plain", lastMessage);
  });
  
  server.begin();
  Serial.println("웹 서버 시작됨.");
}

void loop() {
  server.handleClient();

  // 초음파 센서를 이용한 자동 동작 (수동 모드가 아닐 때)
  long duration;
  float distance;
  
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  distance = (duration * 0.034) / 2;  // cm 단위 거리 계산
  
  if (!manualCommand) {
    if (distance > 0 && distance < distanceThreshold) {
      // 물체 감지 시, 마지막 감지 시간 업데이트 및 게이트 열기(자동)
      lastDetectionTime = millis();
      if (!isOperating) {
        isOperating = true;
        openGate();
        operateSubMotor();
        lastMessage = "OpenService";
      }
    }
  }
  
  // 자동 닫힘: 자동 모드에서만 적용, 감지가 5초 이상 없으면 닫음
  if (!manualCommand && isOperating) {
    if (millis() - lastDetectionTime >= autoCloseDelay) {
      closeGate();
      isOperating = false;
      lastMessage = "Gate Closed";
    }
  }
}

void openGate() {
  gateServo.write(90);  // 서보를 90도로 회전하여 게이트 열기
  // Serial.println("Gate Opened.");  // Serial 대신 웹서버 메시지 사용
}

void closeGate() {
  gateServo.write(0);   // 서보를 0도로 회전하여 게이트 닫기
  // Serial.println("Gate Closed.");
}

void operateSubMotor() {
  // 서브모터 제어 코드 (필요시 실제 제어 코드로 교체)
  // Serial.println("Sub Motor Operating...");
}
