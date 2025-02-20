// buzzer_control.js
console.log("Buzzer Control using ROS IP:", window.ros_ip);  // 이미 설정된 IP를 직접 사용
// ROS ServiceClient 설정
var buzzerControlClient = new ROSLIB.Service({
    ros: ros_pub,
    name: "/Buzzer",
    serviceType: "std_srvs/Trigger"  // 부저를 켜고 끄는 서비스 타입 (트리거 서비스)
});

// 부저 제어 함수
function controlBuzzer(state) {
    var request = new ROSLIB.ServiceRequest({
        data: state  // 1이면 부저 켜기, 0이면 부저 끄기
    });

    buzzerControlClient.callService(request, function(result) {
        console.log("Result from service call:", result.success ? "Success" : "Failure");
        document.getElementById("currentCommand").innerText = state === 1 ? "Buzzer ON" : "Buzzer OFF";
    });

    console.log("Buzzer control state:", state === 1 ? "ON" : "OFF");
}
