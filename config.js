// config.js
window.ros_ip = "192.168.137.54";  // ROS 서버 IP 설정

// ROS Publisher 설정 (한 번만 선언)
window.ros_pub = new ROSLIB.Ros({
    url: `ws://${window.ros_ip}:9090`
});

// 연결 상태 로그
window.ros_pub.on('connection', function() {
    console.log('[Config] Connected to ROS server');
});

window.ros_pub.on('close', function() {
    console.log('[Config] Connection closed');
});
