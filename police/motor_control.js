// ROS 연결 (출력)
const ros_pub = new ROSLIB.Ros({
    url: `ws://${window.ROBOT_IP}:9090`  // config.js에서 설정된 ROBOT_IP 사용
});

// 연결 시, 상태 출력
ros_pub.on('connection', function() {
    console.log('[Publisher] Connected to ROS server');
});

// 연결 종료 시, 상태 출력
ros_pub.on('close', function() {
    console.log('[Publisher] Connection closed');
});

// 모터 제어 명령을 발행하는 함수
function sendMotorCommand(command) {
    const request = new ROSLIB.ServiceRequest({
        command: command
    });

    // 모터 제어를 위한 ROS 토픽 설정
    const motorCommandService = new ROSLIB.Service({
        ros: ros_pub,
        name: "/MotorControl",  // 사용되는 실제 토픽 이름
        serviceType: "jetbotmini_msgs/MotorControl"  // 서비스 타입
    });

    // 명령 발행
    motorCommandService.callService(motorCommand);
    console.log('[Publisher] Command sent:', command);
}
function sendCommand(command) {
    var request = new ROSLIB.ServiceRequest({
        command: command
    });

    motorControlClient.callService(request, function(result) {
        console.log("Result from service call:", result.data);
        document.getElementById("currentCommand").innerText = command;
    });

    console.log("Sent command:", command);
}
