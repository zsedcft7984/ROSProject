// 배터리 제어 및 구독 관련 코드
window.onload = function() {
    // ROS 연결 설정
    const ros_sub = new ROSLIB.Ros({
        url: window.ROSCAR_WS_URL // config.js에서 지정한 WebSocket 주소 사용
    });

    // ROS 연결 성공 시
    ros_sub.on('connection', function() {
        console.log('[Subscriber] Connected to ROS server');
    });

    // ROS 연결 끊겼을 때
    ros_sub.on('close', function() {
        console.log('[Subscriber] Connection closed');
    });

    // 배터리 상태 토픽 구독
    const voltageTopic = new ROSLIB.Topic({
        ros: ros_sub,
        name: window.BATTERY_TOPIC,  // config.js에서 설정한 배터리 토픽 이름 사용
        messageType: 'jetbotmini_msgs/Battery'  // 메시지 타입
    });

    // 배터리 토픽에서 수신된 데이터 처리
    voltageTopic.subscribe(function(message) {
        console.log('[Subscriber] Received voltage:', message.Voltage);
        
        // 화면에 배터리 전압 표시
        document.getElementById('battery-voltage').innerText = 'Voltage: ' + message.Voltage;
    });

    // 구독을 중지하려면 (옵션)
    // voltageTopic.unsubscribe();
};
