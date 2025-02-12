// 카메라 영상 구독 설정
var cameraStream = document.getElementById('cameraStream');

// 카메라 영상 토픽 구독
var imageTopic = new ROSLIB.Topic({
    ros: window.ros_pub,  // config.js에서 정의한 ros_pub 객체 사용
    name: '/cam',         // 카메라 영상 토픽 이름
    messageType: 'sensor_msgs/Image'  // 메시지 타입
});

// 이미지 메시지 수신 처리
imageTopic.subscribe(function(message) {
    // ROS 이미지 메시지를 HTML5 Video에 표시하려면 이미지 데이터를 처리해야 합니다.
    var bytes = new Uint8Array(message.data);
    var blob = new Blob([bytes], {type: 'image/jpeg'});  // 이미지 형식에 맞게 조정
    var url = URL.createObjectURL(blob);
    
    // Video 요소에 재생 URL 설정
    cameraStream.src = url;
});

// 카메라 영상 토픽 연결 상태 로그
imageTopic.on('connection', function() {
    console.log('[Camera Control] Connected to camera topic');
});

imageTopic.on('error', function(error) {
    console.log('[Camera Control] Error:', error);
});

imageTopic.on('close', function() {
    console.log('[Camera Control] Connection to camera topic closed');
});
