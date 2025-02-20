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
    const motorCommand = new ROSLIB.Message({
        data: command
    });

    // 모터 제어를 위한 ROS 토픽 설정
    const motorCommandTopic = new ROSLIB.Topic({
        ros: ros_pub,
        name: "/MotorControl",  // 사용되는 실제 토픽 이름
        serviceType: "jetbotmini_msgs/MotorControl"  // 서비스 타입
    });

    // 명령 발행
    motorCommandTopic.publish(motorCommand);
    console.log('[Publisher] Command sent:', command);
}

// 버튼 클릭 시 호출되는 명령어들
function setupMotorControls() {
    // 버튼에 이벤트 추가
    document.querySelector("#forwardButton").onclick = () => sendMotorCommand('forward');
    document.querySelector("#backwardButton").onclick = () => sendMotorCommand('backward');
    document.querySelector("#leftButton").onclick = () => sendMotorCommand('left');
    document.querySelector("#rightButton").onclick = () => sendMotorCommand('right');
    document.querySelector("#stopButton").onclick = () => sendMotorCommand('stop');
    document.querySelector("#speedFastButton").onclick = () => sendMotorCommand('speed fast');
    document.querySelector("#speedSlowButton").onclick = () => sendMotorCommand('speed slow');
    document.querySelector("#speedNormalButton").onclick = () => sendMotorCommand('speed normal');
}

// 초기화 함수
window.onload = setupMotorControls;
