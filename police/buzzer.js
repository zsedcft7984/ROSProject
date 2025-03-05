// ROS와 WebSocket 연결
const ros_buz = new ROSLIB.Ros({
    url: `ws://${window.ROBOT_IP}:9090`  // WebSocket URL
});

// 부저 제어 서비스 연결
const buzzerService = new ROSLIB.Service({
    ros: ros_buz,
    name: window.BUZZER_SERVICE,  // 부저 서비스 이름
    serviceType: 'jetbotmini_msgs/Buzzer'  // 부저 서비스 타입
});

// 부저 상태를 토글하는 함수
function toggleBuzzer(buzzerState) {
    const request = new ROSLIB.ServiceRequest({
        buzzer: buzzerState  // 부저 상태 (1: 켬, 0: 끔)
    });

    buzzerService.callService(request, function(response) {
        if (response.result) {
            console.log(`Buzzer turned ${buzzerState === 1 ? 'ON' : 'OFF'}`);
        } else {
            console.log('Failed to toggle the buzzer');
        }
    });
}
