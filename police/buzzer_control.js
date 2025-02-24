// ROS 연결을 위한 WebSocket 설정
const ros_buz = new ROSLIB.Ros({
    url: `ws://${window.ROBOT_IP}:9090`  // WebSocket 주소 사용
});

// WebSocket 연결 시 상태 출력
ros_buz.on('connection', function() {
    console.log('[Publisher] Connected to ROS server');
});

// WebSocket 연결 종료 시 상태 출력
ros_buz.on('close', function() {
    console.log('[Publisher] Connection closed');
});

// 부저 제어 서비스 객체 생성
const buzzerService = new ROSLIB.Service({
    ros: ros_buz,
    name: BUZZER_SERVICE,  // 부저 서비스 이름
    serviceType: 'jetbotmini_msgs/Buzzer'  // 부저 서비스 타입
});

// 부저 제어 함수 (1: 켜기, 0: 끄기)
function controlBuzzer(state) {
    const request = new ROSLIB.ServiceRequest({
        buzzer: state  // 'buzzer' 필드를 사용
    });

    buzzerService.callService(request, function(response) {
        if (response.result) {
            console.log('Buzzer response:', response);
            updateBuzzerStatus(state);  // 상태 업데이트
        } else {
            console.log('Buzzer control failed');
            updateBuzzerStatus(null);  // 실패 시 상태 갱신
        }
    });
}

// 부저 상태 업데이트 함수
function updateBuzzerStatus(state) {
    const buzzerStatusElement = document.getElementById("buzzer-status");

    if (state === 1) {
        buzzerStatusElement.textContent = "Buzzer is ON";
    } else if (state === 0) {
        buzzerStatusElement.textContent = "Buzzer is OFF";
    } else {
        buzzerStatusElement.textContent = "Failed to control Buzzer";
    }
}

// 부저 상태 표시 요소 생성
const buzzerStatusElement = document.createElement("p");
buzzerStatusElement.id = "buzzer-status";
document.body.appendChild(buzzerStatusElement);
