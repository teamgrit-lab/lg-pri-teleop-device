# lg-pri-teleop-device

TeamGRIT의 미디어 스트리밍 서버를 활용해 멀티 모달 데이터(영상, 센서, 3D point cloud, 제어 입력)를 네트워크 상에서 주고받을 수 있도록 중계하는 ROS 2 엔드포인트 패키지입니다. 로봇 단의 ROS 토픽과 원격(PC/VR/다른 ROS 시스템) 간의 데이터 전달을 간단히 구성할 수 있습니다.

### 지원 연동 시나리오

- **ROS <-> ROS**: 원격지 ROS 시스템과 주고받기
- **ROS <-> PC**: 브라우저·데스크톱 앱에서 영상/센서 스트림 시청 및 제어 입력 전송
- **ROS <-> VR**: VR 클라이언트로 멀티 모달 스트림 전달 및 입력 반영

### 주요 기능

- **미디어 스트리밍**: `sensor_msgs/Image`데이터를 압축하여 서버로 전송하여 저지연 영상 스트림 제공
- **센서 브리지**: `sensor_msgs/JointState` 등 센서 데이터를 JSON 등으로 변환해 전송
- **3D point cloud 전송**: `sensor_msgs/PointCloud2`를 Draco로 압축해 대역폭 효율적으로 전송
- **제어 입력 구독/발행**: 원격 입력을 ROS 토픽(`sensor_msgs/Joy` 등)으로 재발행
- **동적 런치**: `config/config.yaml`만 수정하면 필요한 노드를 선택적으로 실행

### 빠른 시작

사전 준비

- ROS 2 워크스페이스에 본 패키지가 포함되어 있어야 합니다
- TeamGRIT 미디어 스트리밍 서버의 `host`/`port` 정보
- 로봇관리에서 발급받은 `robot_id`

빌드

```
colcon build
```

만약 hula 관련 별도 메시지 타입을 사용한다면 빌드 전에 다음을 실행하세요

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
  <img  alt="image" src="https://github.com/user-attachments/assets/072f4c16-31a1-4027-b300-160ccc47ca6e" />

#### publish 섹션

- **medias_topics**: 카메라 등 영상 토픽을 서버로 전송합니다. 항목별 필드
  - `name`: 노드 이름
  - `topic`: ROS 영상 토픽(`sensor_msgs/Image`)
  - `endpoint`: 서버 pub 경로(예: `/pang/ws/pub?...&track=left_camera...`) — `#####`는 `robot_id`로 자동 치환
  - `width`/`height`: 전송 해상도. GStreamer 파이프라인에 반영됩니다
- **sensor**: 센서 값을 JSON 등으로 전송합니다
  - 예시 `joint_states`는 `sensor_msgs/JointState`를 읽어 JSON으로 서버에 보냅니다
  - 기본값: `config/config.yaml`의 `sensor`에 `joint_states` 항목이 활성화되어 있으며 `endpoint`의 `track`은 `robot_joint_states`입니다
- **lidar**: 포인트클라우드(`sensor_msgs/PointCloud2`)를 Draco로 압축해 전송합니다
  - `pointcloud_topic1`: 입력 포인트클라우드 토픽
  - TF에서 `base_link`로 변환을 시도하므로 해당 TF가 제공되어야 합니다

##### 토픽별 메시지 타입 표

| 토픽                                   | 메시지 타입               |
| -------------------------------------- | ------------------------- |
| `/camera/camera_right/color/image_raw` | `sensor_msgs/Image`       |
| `/camera/camera_left/color/image_raw`  | `sensor_msgs/Image`       |
| `/camera/camera_head/color/image_raw`  | `sensor_msgs/Image`       |
| `/joint_states`                        | `sensor_msgs/JointState`  |
| `/points`                              | `sensor_msgs/PointCloud2` |

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
      endpoint: "/pang/ws/pub?channel=instant&name=#####&track=robot_joint_states&mode=bundle"
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

##### 구독(subscribe) 토픽별 메시지 타입 표

| 토픽                     | 메시지 타입              |
| ------------------------ | ------------------------ |
| `/joy`                   | `sensor_msgs/Joy`        |
| `/motor_target_position` | `std_msgs/Int32`         |
| `/joint_states`          | `sensor_msgs/JointState` |

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
