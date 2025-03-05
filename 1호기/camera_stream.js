document.addEventListener('DOMContentLoaded', function() {
    // config.js에서 설정된 IP 주소와 ROS 설정을 활용
    const ros_ip = window.ros_ip; // config.js에서 정의된 IP 주소 사용
    const ros = window.ros_pub; // config.js에서 정의된 ROS 연결 객체 사용

    // 카메라 이미지 구독
    const cameraImageTopic = new ROSLIB.Topic({
        ros: ros_pub,
        name: '/cam', // 카메라 영상 토픽 이름
        messageType: 'sensor_msgs/Image'
    });

    // ROS 서버 연결 상태 확인
    ros.on('connection', function() {
        console.log(`[Camera Stream] Connected to ROS server at ws://${ros_ip}:9090`);
    });

    ros.on('close', function() {
        console.log('[Camera Stream] Connection closed');
    });

    // 이미지 처리 함수
    cameraImageTopic.subscribe(function(message) {
        // 메시지의 내용을 콘솔에 출력하여 확인
        console.log("Received Message:");
        console.log("Data:", message.data); // 실제 이미지 데이터
        console.log("Width:", message.width); // 이미지 너비
        console.log("Height:", message.height); // 이미지 높이
        console.log("Encoding:", message.encoding); // 이미지 인코딩 방식
        console.log("Step:", message.step); // 이미지 데이터 한 줄의 바이트 수

        // 웹페이지에 카메라 정보 동적으로 업데이트
        document.getElementById('imageWidth').innerText = message.width;
        document.getElementById('imageHeight').innerText = message.height;
        document.getElementById('imageEncoding').innerText = message.encoding;
        document.getElementById('imageStep').innerText = message.step;
    });
});
