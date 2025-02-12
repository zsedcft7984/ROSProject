window.onload = function() {
    // ✅ 영상 스트리밍 설정
    (function() {
        const videoStream = document.getElementById('videoStream');
        videoStream.src = `http://${window.ros_ip}:8080/stream?topic=/csi_cam_0/image_raw`;
    })();

    // ✅ ROS 전압 정보 구독
    (function() {
        console.log("Subscriber using ROS IP:", window.ros_ip);

        // ROS 연결 설정
        const ros_sub = new ROSLIB.Ros({
            url: `ws://${window.ros_ip}:9090`
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
    })();
};
