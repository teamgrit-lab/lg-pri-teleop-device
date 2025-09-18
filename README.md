# lg-pri

```
cd ~/teamgrit_ws/src/lg-pri-device-py
git pull
colcon build
```

hula와 관련된 별도의 msg 타입이 있다면 colcon build 수행 전에 다음 명령어를 실행해주세요

```
source ~/hula/install/setup.bash
colcon build
```

실행

```
source install/setup.bash
ros2 launch lg_teleop_device lg_teleop_device_launch.py
```

### 설정(config.yaml) 사용 방법

`lg_teleop_device_launch.py`는 패키지의 `share/lg_teleop_device/config/config.yaml`을 읽어 노드를 동적으로 띄웁니다. 설정을 바꾸려면 이 리포지토리의 `config/config.yaml`을 수정한 뒤 다시 빌드(colcon build)하세요.

#### 핵심 개념

- **publish**: 로컬 ROS 토픽을 구독해 인코딩/가공 후 Teleop 서버(WebSocket)로 전송합니다.
- **subscribe**: Teleop 서버에서 수신한 데이터를 로컬 ROS 토픽으로 발행합니다.

#### 공통 상단 설정

- **host**: Teleop 서버 주소(예: `192.168.0.10`)
- **port**: Teleop 서버 포트(예: `8276`)
- **robot_id**: 로봇관리에서 발급받은 로봇 ID. 아래 `endpoint`에 있는 `#####` 자리표시는 런치 시 자동으로 `robot_id`로 치환됩니다.
<img width="2940" height="1666" alt="image" src="https://github.com/user-attachments/assets/3ef88da1-eb41-48a1-8b78-c2523d9d1ae6" />


#### publish 섹션

- **medias_topics**: 카메라 등 영상 토픽을 서버로 전송합니다. 항목별 필드
  - `name`: 노드 이름
  - `topic`: ROS 영상 토픽(`sensor_msgs/Image`)
  - `endpoint`: 서버 pub 경로(예: `/pang/ws/pub?...&track=left_camera...`) — `#####`는 `robot_id`로 자동 치환
  - `width`/`height`: 전송 해상도. GStreamer 파이프라인에 반영됩니다
- **sensor**: 센서 값을 JSON 등으로 전송합니다
  - 예시 `joint_states`는 `sensor_msgs/JointState`를 읽어 JSON으로 서버에 보냅니다
- **lidar**: 포인트클라우드(`sensor_msgs/PointCloud2`)를 Draco로 압축해 전송합니다
  - `pointcloud_topic1`: 입력 포인트클라우드 토픽
  - TF에서 `base_link`로 변환을 시도하므로 해당 TF가 제공되어야 합니다

예시(publish 일부만 발췌):

```yaml
host: "192.168.0.10"
port: "8276"
robot_id: "my_robot_001"
publish:
  medias_topics:
    - name: left_camera
      topic: /camera/left/image_raw
      endpoint: "/pang/ws/pub?channel=instant&name=#####&track=left_camera&mode=bundle"
      width: "640"
      height: "480"
  sensor:
    - name: joint_states
      topic: /joint_states
      endpoint: "/pang/ws/pub?channel=instant&name=#####&track=joint_states&mode=bundle"
```

#### subscribe 섹션

- **control_topics**: 서버에서 수신한 제어 입력을 로컬 토픽으로 발행합니다. 항목별 예시
  - `gamepad_control`: 서버 JSON을 `sensor_msgs/Joy`로 변환해 `gamepad_control_topic`에 발행
    - 필드: `topic`(발행 토픽), `endpoint`
  - `waist_control`: 서버 입력을 기준으로 목표 위치(`std_msgs/Int32`)를 `waist_control_topic`에 발행
    - 필드: `topic`(발행 토픽), `endpoint`, `waist_control_speed_mm_per_sec`
    - 피드백 토픽 `/motor_current_position`(Int32)을 구독해 현재 위치를 반영합니다
  - `left_arm`/`right_arm`: `tf_target_frame`을 기준으로 팔 제어(세부는 로봇별 구현에 의존)
  - `head`: 헤드 제어 입력 처리
  - `joint_states`: 서버에서 받은 조인트 상태를 로컬 토픽으로 재발행

예시(subscribe 일부만 발췌):

```yaml
subscribe:
  control_topics:
    - name: gamepad_control
      topic: /joy
      endpoint: "/pang/ws/pub?channel=instant&name=#####&track=amr&mode=bundle"
    - name: waist_control
      topic: /motor_target_position
      endpoint: "/pang/ws/pub?channel=instant&name=#####&track=waist&mode=bundle"
      waist_control_speed_mm_per_sec: 20.0
```

#### 사용 절차

1. `config/config.yaml`에서 `host`, `port`, `robot_id`를 채웁니다
2. 필요한 `publish`/`subscribe` 항목의 주석을 해제하고 토픽/엔드포인트를 알맞게 입력합니다
3. 빌드 및 실행
   - `colcon build`
   - `source install/setup.bash`
   - `ros2 launch lg_teleop_device lg_teleop_device_launch.py`

참고 사항

- 런치는 `config.yaml`의 최상위 `host`/`port`를 각 노드 파라미터로 전달하고, 각 항목의 `endpoint`에서 `#####`를 `robot_id`로 치환합니다
- 토픽 존재 여부는 `ros2 topic list`, 메시지 타입은 `ros2 topic info`로 확인할 수 있습니다
- LiDAR는 프레임 변환(`base_link` 기준)과 대역폭을 고려해 포인트 수와 해상도를 조정하세요
