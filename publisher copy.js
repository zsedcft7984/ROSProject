// publisher.js
const ros_ip = window.ros_ip;  // config.js에서 설정한 IP를 전역 변수로 사용

console.log("[Publisher] Using ROS IP:", window.ros_ip); // window.ros_ip 직접 출력

// ROS 연결 설정
const ros_pub = new ROSLIB.Ros({
    url: `ws://${window.ros_ip}:9090`  // window.ros_ip 사용
});

ros_pub.on('connection', function() {
    console.log('[Publisher] Connected to ROS server');
});

ros_pub.on('close', function() {
    console.log('[Publisher] Connection closed');
});

// /Motor 서비스 호출을 위한 Publisher
const motorService = new ROSLIB.Service({
    ros: ros_pub,
    name: '/Motor',
    serviceType: 'jetbotmini_msgs/Motor'
});

// 로봇 이동 명령을 보내는 함수
function sendVelocity(leftSpeed, rightSpeed) {
    const motorRequest = new ROSLIB.ServiceRequest({
        leftspeed: leftSpeed,
        rightspeed: rightSpeed
    });

    motorService.callService(motorRequest, function(response) {
        console.log('[Publisher] Motor command sent:', response.result);
    });
}

// HTML 버튼이 로드된 후 이벤트 리스너 추가
document.addEventListener("DOMContentLoaded", function () {
    const moveForwardBtn = document.getElementById('moveForward');
    if (moveForwardBtn) {
        moveForwardBtn.onclick = function() {
            sendVelocity(1, 1);  // 전진
        };
    }

    const stopBtn = document.getElementById('stop');
    if (stopBtn) {
        stopBtn.onclick = function() {
            sendVelocity(0.0, 0.0);  // 정지
        };
    }
});
