// 로봇 IP 주소
const ROBOT_IP = '192.168.137.64';  // 차량의 IP 주소

// WebSocket 연결 URL
const ROSCAR_WS_URL = `ws://${ROBOT_IP}:9090`;  // WebSocket 연결 주소 (9090 포트)

// 카메라 스트리밍 URL
const CAMERA_FEED_URL = `http://${ROBOT_IP}:8080`;  // 영상 스트리밍 URL (8080 포트)


//서비스 이름
const MOTOR_CONTROL_SERVICE = `/MotorControl`;  // 모터 제어 서비스
const BUZZER_SERVICE = `/Buzzer`;  // 부저 제어 서비스 (서비스 이름을 수정)
const BATTERY_TOPIC = `/voltage`;  // 배터리 상태 토픽

// 배터리 관련 변수 (배터리 상태 토픽의 기본 값)
const DEFAULT_BATTERY_VOLTAGE = 0.0;

// 라인트레이싱 제어 서비스
const LINE_TRACING_SERVICE = `/SetAuto`;  // 라인트레이싱 제어 서비스

// 전역 객체에 설정값 할당
window.ROBOT_IP = ROBOT_IP;
window.ROSCAR_WS_URL = ROSCAR_WS_URL;
window.MOTOR_CONTROL_SERVICE = MOTOR_CONTROL_SERVICE;
window.BUZZER_SERVICE = BUZZER_SERVICE;  // 수정된 BUZZER_SERVICE 값 반영
window.BATTERY_TOPIC = BATTERY_TOPIC;
window.DEFAULT_BATTERY_VOLTAGE = DEFAULT_BATTERY_VOLTAGE;
window.LINE_TRACING_SERVICE = LINE_TRACING_SERVICE;
window.CAMERA_FEED_URL = CAMERA_FEED_URL;
