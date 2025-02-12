// ROS 연결 설정
var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'  // ROSBridge WebSocket 주소
});

ros.on('connection', function() {
    console.log('Connected to ROSBridge WebSocket server.');
});

ros.on('error', function(error) {
    console.error('Error connecting to ROS:', error);
});

ros.on('close', function() {
    console.log('Connection to ROS closed.');
});

// ROS ServiceClient 설정
var motorControlClient = new ROSLIB.Service({
    ros: ros,
    name: '/MotorControl',  // ROS 서비스 이름
    serviceType: 'std_msgs/String'  // 서비스 타입
});

// 명령 전송 함수
function sendCommand(command) {
    var request = new ROSLIB.ServiceRequest({
        data: command
    });

    motorControlClient.callService(request, function(result) {
        console.log('Result from service call:', result.data);  // 서비스 호출 결과 출력
    });

    console.log('Sent command:', command);
}
