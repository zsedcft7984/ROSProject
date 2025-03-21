# APT - 스마트 아파트 자동 관리 시스템

## 프로젝트 개요

이 프로젝트는 로봇 운영 체제(ROS)를 활용하여 자율 주행, 객체 인식, 웹페이지와 연계, 아두이노 동작을 구현하는 것을 목표로 합니다. `Final` 폴더에는 최종 구현된 코드와 관련 자료가 포함되어 있습니다.

## 주요 기능

- **자율 주행**: 라인트레이싱을 기반으로 하여 자율 주행을 구현하였습니다.
- **객체 인식**: YOLO 모델을 사용하여 객체를 검출하고, 검출된 객체에 따라 서버에 이미지가 저장되며 ROS에서 특정 서비스를 동작하게 합니다.
- **웹페이지와 연계**: 웹페이지에서 ROS의 동작을 Service와 Topic을 사용하여 제어하거나 특정 동작을 수행하도록 명령할 수 있습니다.
- **아두이노 동작**: Wi-Fi로 연결된 아두이노가 PC에서 명령을 받아 동작하고, 동작 결과를 다시 PC로 전송합니다.

## 설치 및 실행 방법
### 설치 조건 
ROS
- ROS Melodic 버전을 사용하는 보드(Jetson nano 4GB)를 기반으로 합니다.
- http://www.yahboom.net/study/JETBOT-mini 에서 Download 에 Jetbot MINI image 4GB를 다운받아서 balenaEtcher을 통해 파일을 적용시킨 USB가 필요합니다(32GB 이상)
- ROS 동작에 관한 추가적인 사항은 Tip 폴더의 내용을 참고하세요 

Arduino
- 사용하는 보드는 아두이노 우노+WIFI D1 R1 보드(ESP8266)를 사용합니다.
- 아두이노 환경설정 - 추가 보드메니져 URL 에다가 https://arduino.esp8266.com/stable/package_esp8266com_index.json 를 입력하여 다운 받습니다.
- 사용하는 포트의 보드설정에  LOLIN(WeMos) D1 R1 를 선택하여 사용합니다.

YOLO
- Visual Studio Code를 사용해서 편집했습니다.
- yolo 폴더 내부에 istall목록.txt 참고하면 됩니다.
  
1. **프로젝트 클론**

   ```bash
   git clone https://github.com/zsedcft7984/ROSProject.git
   ```

2. **워크스페이스 설정**

   ```bash
   cd ROSProject/Final
   catkin_make
   ```

3. **환경 설정**

   ```bash
   source devel/setup.bash
   ```

## 개인 프로젝트 기여

### 1. ROS
- ROS 장치는 Yahboom 회사에서 제공하는 Jetbot Mini를 사용합니다. 
- 장치에는 자체 카메라, DC 모터, 부저, LED 등이 제공됩니다. 프로젝트에서는 추가적으로 웹캠을 부착하여 동작합니다.
- jetbotmini_driver.py에서 ROS의 다양한 기능을 동작하게 하는 서비스와 토픽이 정의되어 있습니다.
- 제공하는 기능은 Auto mode on/off, Motor 제어, 부져 제어, 베터리 정보 제공 이 있습니다.
- 장치는 내장된 카메라의 영상을 기반으로 라인트레이싱을 수행하여 자율 주행을 구현하였습니다.
- 대부분의 서비스 및 토픽은 Wi-Fi로 연결된 PC에서 제어하기 위해 JavaScript 파일을 제작하였으며, HTML 파일에서 버튼을 클릭하면 원하는 서비스가 동작하도록 구현하였습니다.
- 사용하는 Java Script 파일이 다양하기 때문에 일괄적으로 관리하기 위한 config.js를 제작하여 ip같은 정보가 변경되었을때 수정하기 편리하도록 제작하였습니다.
- 동작간 모터의 경우 정지되는 동작이 많아 모터의 정지를 관여하는 함수를 따로 제작하고 정지되어 있는 상태를 기록하는 변수를 추가로 설정하였습니다.

### ROS 서비스 및 퍼블리셔 목

<table>
    <tr>
        <th>기능</th>
        <th>`서비스/토픽 이름`</th>
        <th>설명</th>
    </tr>
    <tr>
        <td><b>Auto Mode</b></td>
        <td>`/SetAuto`</td>
        <td>자율 주행 모드를 활성화/비활성화합니다.</td>
    </tr>
    <tr>
        <td><b>Motor 제어</b></td>
        <td>`/MotorControl`</td>
        <td>속도와 방향을 제어할 때 사용됩니다.</td>
    </tr>
    <tr>
        <td><b>부저 제어</b></td>
        <td>`/Buzzer`</td>
        <td>부저를 켜거나 끌 수 있습니다.</td>
    </tr>
    <tr>
        <td><b>배터리 정보 제공</b></td>
        <td>`/voltage`</td>
        <td>현재 배터리 상태를 확인합니다.</td>
    </tr>
</table>

### 2. YOLO 모델 인식 프로그램
- 학습된 모델을 적절히 사용하여 roslip을 사용해 ros에 명령을 보내고 와 AWS에 저장되는 기능을 하도록 제작하였습니다.
- 추가적으로 출입구를 구현하기 위해 아두이노와 연계하여 특정 동작이 있을때 AWS에 저장되고 서보 모터가 동작하도록 제작하였습니다.

### 3. Arduino
- Wi-Fi를 통해 웹 서버와 정보를 주고 받습니다.
- 웹에서 아두이노로 명령을 보내고 동작을 하게되면 log를 yolo모델이 적용된 pc에 보내주어 특정 log가 오면 추가 명령을 보내도록 제작했습니다.
- 초음파 센서를 활용하여 일정 거리 내에 물체가 감지되면 서보 모터가 자동으로 동작하여 자동 개폐 기능을 구현하였습니다.

