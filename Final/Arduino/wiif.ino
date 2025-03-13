#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

const char* ssid = "apt2"; // wifi 이름
const char* password = "123456789"; // wifi 비밀번호

ESP8266WebServer server(80);
Servo gateServo;

// 차단바 열기 및 닫기 함수 선언
void openGate();
void closeGate();

void setup() {
  Serial.begin(115200);              // 시리얼 통신 시작
  gateServo.attach(D5, 700, 2800);     // 서보 핀 설정 및 펄스 범위 지정

  // Wi‑Fi 연결
  WiFi.begin(ssid, password);
  Serial.print("WiFi 연결 중...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("연결됨, IP 주소: ");
  Serial.println(WiFi.localIP());

  // 웹서버 엔드포인트 등록
  server.on("/gate_control", HTTP_GET, [](){
    String command = server.arg("cmd");
    command.trim();
    
    if (command.equalsIgnoreCase("raise")) {
      openGate();
      // 차단바를 연 후 5초 후 자동으로 닫기
      delay(7000);
      closeGate();
      server.send(200, "text/plain", "Gate Raised then Closed.");
    } 
    else {
      server.send(400, "text/plain", "Invalid Command.");
    }
  });

  // 웹서버 시작
  server.begin();
  Serial.println("웹 서버 시작됨.");
}

void loop() {
  server.handleClient();  // 클라이언트 요청 처리
}

void openGate() {
  gateServo.write(90);  // 서보를 90도로 설정하여 차단바 올리기
  Serial.println("Gate Opened.");
}

void closeGate() {
  gateServo.write(0);   // 서보를 0도로 설정하여 차단바 내리기
  Serial.println("Gate Closed.");
}
