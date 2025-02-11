// subscriber.js (전압 정보 수신)

console.log("Subscriber using ROS IP:", window.ros_ip);  // window.ros_ip로 변경

// ROS 연결 설정
const ros_sub = new ROSLIB.Ros({
    url: `ws://${window.ros_ip}:9090`  // window.ros_ip 사용
});

ros_sub.on('connection', function() {
    console.log('[Subscriber] Connected to ROS server');
});

ros_sub.on('close', function() {
    console.log('[Subscriber] Connection closed');
});

// /voltage 토픽 구독
const voltageTopic = new ROSLIB.Topic({
    ros: ros_sub,
    name: '/voltage',
    messageType: 'jetbotmini_msgs/Battery'
});

voltageTopic.subscribe(function(message) {
    console.log('[Subscriber] Received voltage:', message.Voltage);
    document.getElementById('voltageDisplay').innerText = 'Voltage: ' + message.Voltage;
});
