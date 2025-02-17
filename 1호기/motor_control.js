// motor_control.js

console.log("[Publisher] Using ROS IP:", window.ros_ip);  // 이미 설정된 IP를 직접 사용

// ROS ServiceClient 설정
var motorControlClient = new ROSLIB.Service({
    ros: ros_pub,
    name: "/MotorControl",
    serviceType: "jetbotmini_msgs/MotorControl"
});

// 명령 전송 함수
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
