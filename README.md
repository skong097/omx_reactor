# OMX Reactor

카메라 앞 사람의 **감정/제스처에 OpenManipulator-X 가 Gazebo 시뮬에서 실시간 표현 모션으로 반응**하는 데모. ROS 2 Jazzy 기반, 단일 터미널 launch, 단일 페이지 웹 대시보드.

```
[웹캠] -> mediapipe (face V·A / hand gesture)
       -> rapport EMA + 감정 분면 + gesture event
       -> motion mapper (priority + cooldown + 큐 깊이 1 scheduler)
       -> OMX action (arm_controller / gripper_controller)
       -> FastAPI + WebSocket -> 단일 페이지 대시보드 (V·A 4분면 + 추이 그래프 + Gazebo MJPEG + 모션 이벤트)
```

## 한 줄 요약

```
얼굴 표정 (V·A 4분면) + 손 제스처 (mediapipe Gesture Recognizer 7종 + thumb-index distance) -> 18 표현 모션 (감정 분면 4 + 세션 인사 2 + 제스처 mimic 10 + 그리퍼 2)
```

## 모션 라이브러리 (현 18종)

| Trigger | Event | Motion | Priority |
|---|---|---|---:|
| **감정 4분면** | V·A coord | DANCE / FREEZE / CONSOLE / IDLE | 10 / 0 |
| **세션 경계** | new_track / track_gone | HELLO / BYE | 100 |
| **gesture: Open_Palm + wrist 중간** | hand_visible | HAND_OUT | 80 |
| **gesture: Open_Palm + 중간 + wave** | twinkle | TWINKLE (base 좌우) | 82 |
| **gesture: Open_Palm + 위** | hands_up | HANDS_UP (팔 위로 쭉) | 85 |
| **gesture: Open_Palm + 위 + wave** | hands_up_wave | HANDS_UP_WAVE (만세+흔듦) | 90 |
| **gesture: Pointing_Up** | pointing_up | POINT_BACK | 82 |
| **gesture: Thumb_Up** | thumb_up | NOD (끄덕) | 82 |
| **gesture: Thumb_Down** | thumb_down | SAD | 82 |
| **gesture: Victory** | victory | CHEER | 85 |
| **gesture: ILoveYou** | ilove_you | HEART | 82 |
| **gesture: Closed_Fist** | closed_fist | STRONG | 82 |
| **gesture: thumb-index distance > 0.20** | gripper_open | GRIPPER_OPEN | 95 |
| **gesture: thumb-index distance < 0.05** | gripper_close | GRIPPER_CLOSE | 95 |

새 motion 추가 = `motions.py` 의 `MOTIONS` 리스트에 한 항목 + `trajectories.py` 에 trajectory factory 1개. mapper / scheduler 0 수정 (plug-in 구조).

## Architecture

```
+-------------------------+         +------------------------------+
|  v4l2_camera (webcam)   |-------->|  geva_node (vendored)        |
|  /webcam/image_raw      |   |     |  mediapipe FaceLandmarker    |
+-------------------------+   |     |  -> /emotion/state (V·A)     |
                              |     +--------------+---------------+
                              |                    |
                              |                    v
                              |     +------------------------------+
                              |     |  rapport_tracker (vendored)  |
                              |     |  EMA + hysteresis            |
                              |     |  -> /rapport/event           |
                              |     +--------------+---------------+
                              v                    |
        +----------------------------+             |
        |  gesture_detector_node     |             |
        |  mediapipe Gesture         |             |
        |  Recognizer + WaveDetector |             |
        |  -> /gesture/event         |             |
        +--------------+-------------+             |
                       |                           |
                       v                           v
                 +---------------------------------------+
                 |  reactor_node                         |
                 |  Context (emotion + gesture + session)|
                 |  -> motion_mapper.select_motion       |
                 |  -> motion_scheduler (priority/queue) |
                 |  -> arm_sender / gripper_sender       |
                 +-------------------+-------------------+
                                     |
        +----------------------------+----------------------------+
        |                            |                            |
        v                            v                            v
  +-----------+              +--------------+           +------------------------+
  |  Gazebo   |              |  OMX arm     |           |  dashboard_node        |
  |  external |              |  /gripper    |           |  /ws/v1/engaging       |
  |   cam     |              |  controllers |           |  /api/gazebo_view.mjpg |
  |  MJPEG    |              |  (action)    |           |  -> 브라우저 (1x2 UI) |
  +-----------+              +--------------+           +------------------------+
```

## Prerequisites

| 종류 | 항목 | 비고 |
|---|---|---|
| OS | Ubuntu 24.04 | |
| ROS | ROS 2 Jazzy | desktop 권장 |
| ROBOTIS | open_manipulator stack | apt 또는 `~/robot_arm/` source 빌드 |
| Python | mediapipe 0.10.14, fastapi, uvicorn, websockets | `requirements.txt` |
| 시스템 | numpy 1.26.4, opencv 4.6.0, cv_bridge, v4l2_camera | apt — mediapipe 가 user site 에 numpy 2.x / opencv-* 끌어옴, **cleanup 필수** |
| 하드웨어 | USB 웹캠 (`/dev/video0`) | gesture 인식 |
| GPU | OpenGL 호환 (ogre2 rendering) | Gazebo 가상 카메라 sensor 필요 |

### 외부 부트스트랩 (한 번)

#### ROBOTIS world SDF 의 Sensors plugin 추가 (Gazebo 가상 카메라 작동 필수)

`open_manipulator_bringup/worlds/empty_world.sdf` 의 plugin 블록에 한 줄 추가:

```xml
<plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'>
  <render_engine>ogre2</render_engine>
</plugin>
```

ROBOTIS 의 default world 에는 Sensors plugin 빠져 있어 dashboard 의 OMX MJPEG view 가 작동 안 함.

#### mediapipe 모델 다운로드

`src/vendored/README.md` 의 외부 모델 부트스트랩 절차 참고 (face_landmarker / efficientdet_lite0 / gesture_recognizer / hand_landmarker — 총 4 파일, ~23MB).


## Real Hardware (omx_f follower)

### 사전 확인
1. follower OpenRB-150 만 USB 연결 (leader 는 띄우지 않음)
2. `ls /dev/serial/by-id/` 로 by-id symlink 확인 → `omx_real.launch.py` 의 default port 와 일치 시 그대로, 다르면 `--port=` 명시
3. ROBOTIS `open_manipulator_bringup` 가 source 빌드 또는 apt 설치되어 있어야 함

### 첫 동작 검증 (필수)
풀 데모 전 단발 sanity:

```bash
# 터미널 A
ros2 launch omx_reactor omx_real.launch.py

# 터미널 B (init_position 끝나면)
OMX_ROBOT=real python3 scripts/sanity_real.py
```

IDLE → CONSOLE 가 안전한 진폭으로 동작하는지 + joint5 가 0 유지하는지 확인 후 풀 데모로 진입.

### sim vs real 자동 분기
- `--robot=sim` (default): Gazebo + 4축 + `gripper_left_joint` + dashboard MJPEG=`/external_cam/image`
- `--robot=real`: omx_f bringup + 5축 (joint5=0) + `gripper_joint_1` + dashboard MJPEG=`/webcam/image_raw`
- 분기 메커니즘: `OMX_ROBOT` 환경 변수 (launch 가 자동 set) + `mjpeg_input_topic` ROS param

### 비상 정지
OpenRB-150 USB 케이블 분리 (전원 차단) 또는 `Ctrl+C`.


## Installation

  ```bash
  git clone https://github.com/<user>/omx-reactor.git ~/omx_reactor
  cd ~/omx_reactor

  # 1. Python 의존성 (mediapipe 가 user-site 에 numpy 2.x / opencv-* 끌어오므로 cleanup 필수)
  pip install --user -r requirements.txt
  pip uninstall --yes numpy opencv-python opencv-python-headless opencv-contrib-python
  python3 -c "import numpy, cv2;
  print(numpy.__version__, cv2.__version__)"   # 1.26.4 / 4.6.0

  # 2. mediapipe 모델 부트스트랩 (4파일, ~23 MB — *.task / *.tflite 는 .gitignore 대상)
  #    빌드 이전에 받아야 colcon 의  data_files 가 install/share 에 배치합니다.
  
  MODELS_DIR="src/vendored/dobi_npc_emotion/models"
  GESTURE_DIR="src/omx_reactor/models/gesture"
  
  mkdir -p "$MODELS_DIR" "$GESTURE_DIR"
  
  curl -fsSL https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/latest/face_landmarker.task -o "$MODELS_DIR/face_landmarker.task"
  
  curl -fsSL https://storage.googleapis.com/mediapipe-models/object_detector/efficientdet_lite0/int8/latest/efficientdet_lite0.tflite -o "$MODELS_DIR/efficientdet_lite0.tflite"
  
  curl -fsSL https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task -o "$GESTURE_DIR/gesture_recognizer.task"
  
  curl -fsSL https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task -o "$GESTURE_DIR/hand_landmarker.task"
  
  ls -lh "$MODELS_DIR" "$GESTURE_DIR"
  
  # 4개 파일: 3.6M / 4.4M / 8.0M / 7.5M
  
  
  
# 빌드
source /opt/ros/jazzy/setup.bash
source ~/robot_arm/install/setup.bash   # 또는 apt 의 ros-jazzy-open-manipulator-*
colcon build --symlink-install
source install/setup.bash
```

# Python
pip install --user -r requirements.txt
pip uninstall --yes numpy opencv-python opencv-python-headless opencv-contrib-python
python3 -c "import numpy, cv2; print(numpy.__version__, cv2.__version__)"   # 1.26.4 / 4.6.0

## Run

```bash
# 데모 (단일 터미널 — Ctrl+C 한 번에 자식 다 종료)
bash scripts/run_demo.sh                            # default: sim (Gazebo) + v4l2 카메라
bash scripts/run_demo.sh --camera=file --file-path=samples/x.mp4
bash scripts/run_demo.sh --camera=external          # 카메라는 외부 launch

# 실 OMX (omx_f follower) — 반드시 sanity 먼저
bash scripts/run_demo.sh --robot=real               # default port = by-id symlink
bash scripts/run_demo.sh --robot=real --port=/dev/ttyACM0

# 종료 (해당 터미널 Ctrl+C — 잔존 청소 + 카메라 device handle release 는 stop_demo)
bash scripts/stop_demo.sh
```

대시보드: <http://localhost:8800/>

## Demo Scenarios

### 감정 인식
| 표정 | 분면 | 모션 |
|---|---|---|
| 미소 | Q1 (V+ A+) | DANCE — base swing |
| 무표정 | deadband | IDLE |
| 슬픈 | Q3 (V- A-) | CONSOLE — joint4 위아래 부드러운 |
| 화난 | Q2 (V- A+) | FREEZE — joint2 살짝 down |

### 세션 인사
- 카메라 앞 새 등장 -> HELLO
- 3 초 자리 비움 -> BYE

### 손 제스처 (mediapipe Gesture Recognizer + Open_Palm 자세 별 분기)
| 제스처 | 모션 |
|---|---|
| 손바닥 펴기 (Open_Palm, 중간 높이) | HAND_OUT — 손 내밀기 mimic |
| 손바닥 펴기 + 좌우 흔듦 (중간 높이) | TWINKLE — 반짝반짝 (base 좌우) |
| 손바닥 펴기 + 위로 (만세 자세) | HANDS_UP — 팔 위로 쭉 |
| 만세 + 좌우 흔듦 | HANDS_UP_WAVE — 만세 + 인사 |
| 검지 위 (Pointing_Up) | POINT_BACK — 한쪽 가리킴 |
| 엄지 척 (Thumb_Up) | NOD — 끄덕 |
| 엄지 아래 (Thumb_Down) | SAD — 머리 숙이듯 |
| V 사인 (Victory) | CHEER — joint1 좌우 빠르게 |
| 록 사인 (ILoveYou) | HEART — 부드러운 좌우 |
| 주먹 (Closed_Fist) | STRONG — joint1 한쪽 + 위로 hold |
| 엄지+검지 벌림 (distance > 0.20) | GRIPPER_OPEN |
| 엄지+검지 모음 (distance < 0.05) | GRIPPER_CLOSE |

### 대시보드 (1x2 반응형)
- **왼쪽** — 감정 분석 (V·A 좌표 + circumplex SVG 4분면 + EMA 시계열 그래프 + rapport 카운터)
- **오른쪽** — OMX 모션 (Gazebo MJPEG view + zoom slider 0.5x ~ 3.0x + 실시간 모션 이벤트 list 15개)
- **반응형** — 폭 < 768px 시 1 column 으로 wrap

## Project Structure

```
omx_reactor/
├── README.md                      # 본 파일
├── requirements.txt
├── LICENSE                        # Apache-2.0
├── scripts/
│   ├── run_demo.sh                # 단일 터미널 launch
│   └── stop_demo.sh               # 잔존 청소 + 카메라 release
└── src/
    ├── omx_reactor/               # 신규 코드 (본 프로젝트)
    │   ├── omx_reactor/
    │   │   ├── context.py             # EmotionSignal / GestureSignal / Context dataclass
    │   │   ├── motion_mapper.py       # Context -> Motion (priority 내림차순 첫 매치)
    │   │   ├── motion_scheduler.py    # cooldown + priority interrupt + 큐 깊이 1
    │   │   ├── session_tracker.py     # new_track / track_gone 이벤트
    │   │   ├── gesture_detection.py   # classify_hand_state + WaveDetector (pure logic, TDD)
    │   │   ├── gesture_detector_node.py
    │   │   ├── motions.py             # MOTIONS plug-in 리스트
    │   │   ├── trajectories.py        # 18 trajectory factory (arm + gripper)
    │   │   ├── omx_trajectory_sender.py
    │   │   ├── reactor_node.py        # 모든 신호 합성 -> motion dispatch
    │   │   ├── dashboard_node.py      # FastAPI + WebSocket (/ws/v1/engaging + MJPEG)
    │   │   └── web/static/            # index.html + engaging-analytics.js (vendored) + app.js
    │   ├── launch/
    │   ├── models/                    # external_cam SDF + gesture model
    │   └── test/                      # 131 단위 테스트 (TDD)
    └── vendored/                  # doby_controller 발췌 (0 modification — 추후 본 prj 가 doby 로 merge 시 통째 폐기)
        ├── README.md              # 출처 + 모델 부트스트랩 + 수정 정책
        ├── dobi_npc_msgs/         # EmotionState + RapportEvent + PersonTrack(Array) msg
        └── dobi_npc_emotion/      # geva_node + rapport_tracker_node + 모델
```

## Development

### 단위 테스트 (131 tests, pure logic 우선 TDD)
```bash
cd src/omx_reactor && python3 -m pytest test/ -v
```

| 모듈 | 테스트 수 | 검증 |
|---|---:|---|
| motion_mapper | 9 | priority 내림차순 + trigger 매치 |
| motion_scheduler | 10 | cooldown / interrupt / 큐 depth 1 |
| session_tracker | 8 | new_track / track_gone / grace |
| trajectories | 73 | joint name / 시간 단조 / ±1.2rad 안전 / velocity ≤2.5rad/s |
| gesture_detection | 21 | visibility cooldown / classify / wave |

## Roadmap / Status

### Phase 1 — UI + 데이터 통합 (완료)
- doby_controller 의 모객 분석 UI carry-over (V·A circumplex + EMA 시계열 그래프 + rapport counters)
- 1x2 반응형 대시보드
- OMX Gazebo MJPEG view 임베드 + zoom slider
- 모션 이벤트 실시간 list

### Phase 1.5 — 제스처 mimic (완료)
- mediapipe Gesture Recognizer 통합
- 10 제스처 mimic 모션 (HAND_OUT / TWINKLE / HANDS_UP* / POINT_BACK / NOD / SAD / CHEER / HEART / STRONG)
- thumb-index distance 기반 그리퍼 제어

### Phase 2 — Customer ReID + 영속 기록 (보류)
- 얼굴 ReID 로 영속 customer_id 부여 (insightface ArcFace)
- customer 별 EMA 누적 + reaction_history
- MySQL 적재 + 30 일 retention purge

## Acknowledgments

본 데모의 감정 인식 파이프라인 (`geva_node`, `rapport_tracker`) + 모객 분석 대시보드 컴포넌트 (engaging-analytics.js + V·A circumplex SVG + engagement-timeline) 는 별 프로젝트인 cafe NPC funnel BT (PinkLAB internal `doby_controller`) 에서 발췌 — `src/vendored/` 디렉토리, 0 수정 카피, 출처는 `src/vendored/README.md` 참조.

OMX URDF + 컨트롤러 + Gazebo bringup 은 ROBOTIS 공식 [open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator) 사용.

## License

Apache-2.0
