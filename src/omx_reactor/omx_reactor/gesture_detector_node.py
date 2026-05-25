"""gesture_detector_node — /webcam/image_raw -> mediapipe GestureRecognizer
-> /gesture/event (std_msgs/String JSON).

GestureRecognizer 가 default 7 gesture (Open_Palm, Closed_Fist, Pointing_Up,
Thumb_Up/Down, Victory, ILoveYou) 자동 인식 + 21 hand landmark 반환.

본 노드는:
  1. gesture name 추출
  2. wrist landmark (idx 0) 의 normalized x/y 추출
  3. WaveDetector 로 wrist x 시계열 진동 감지 (좌우 흔듦)
  4. classify_hand_state(gesture, wrist_y, is_waving) → event_type 결정
  5. per-event_type cooldown 후 /gesture/event 로 publish

event_type:
  - 'hand_visible'      — Open_Palm + wrist 중간 (default)
  - 'hands_up'          — Open_Palm + wrist 위쪽
  - 'hands_up_wave'     — hands_up + 좌우 진동
  - (다른 gesture 는 추후 추가 — 현재 None 반환)

모델: share/omx_reactor/models/gesture/gesture_recognizer.task (외부 부트스트랩).
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from omx_reactor.gesture_detection import WaveDetector, classify_hand_state


class GestureDetectorNode(Node):
    """mediapipe GestureRecognizer 로 손 인식 + classify + per-event cooldown."""

    def __init__(self):
        super().__init__('gesture_detector_node')
        self.declare_parameter('image_topic', '/webcam/image_raw')
        self.declare_parameter('event_topic', '/gesture/event')
        self.declare_parameter('cooldown_sec', 5.0)
        self.declare_parameter('model_path', '')
        self.declare_parameter('up_threshold', 0.4)
        self.declare_parameter('wave_window_size', 10)
        self.declare_parameter('wave_oscillation_threshold', 0.15)

        img_topic = str(self.get_parameter('image_topic').value)
        event_topic = str(self.get_parameter('event_topic').value)
        self._cooldown = float(self.get_parameter('cooldown_sec').value)
        model_path = str(self.get_parameter('model_path').value)
        self._up_threshold = float(self.get_parameter('up_threshold').value)
        window = int(self.get_parameter('wave_window_size').value)
        osc = float(self.get_parameter('wave_oscillation_threshold').value)

        if not model_path:
            model_path = str(
                Path(get_package_share_directory('omx_reactor'))
                / 'models' / 'gesture' / 'gesture_recognizer.task'
            )

        # mediapipe Tasks API — GestureRecognizer
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision

        base_options = mp_python.BaseOptions(model_asset_path=model_path)
        options = vision.GestureRecognizerOptions(
            base_options=base_options,
            num_hands=1,
            min_hand_detection_confidence=0.5,
        )
        self._recognizer = vision.GestureRecognizer.create_from_options(options)
        self._mp_image_cls = mp.Image
        self._mp_image_fmt = mp.ImageFormat.SRGB

        self._wave = WaveDetector(
            window_size=window, oscillation_threshold=osc)
        self._last_emit_type: str | None = None
        self._last_emit_t: float = 0.0

        self._bridge = CvBridge()
        self.create_subscription(Image, img_topic, self._on_image, 1)
        self._pub = self.create_publisher(String, event_topic, 10)

        self.get_logger().info(
            f'gesture_detector_node ready — {img_topic} -> {event_topic}, '
            f'cooldown={self._cooldown}, up_threshold={self._up_threshold}, '
            f'wave(window={window}, osc={osc}), model={model_path}'
        )

    def _on_image(self, msg: Image):
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            rgb = bgr[:, :, ::-1].copy()
            mp_img = self._mp_image_cls(image_format=self._mp_image_fmt, data=rgb)
            result = self._recognizer.recognize(mp_img)
        except Exception as e:
            self.get_logger().warning(
                f'GestureRecognizer recognize 실패: {e}',
                throttle_duration_sec=5.0,
            )
            return

        if not result.hand_landmarks:
            return

        # gesture name + wrist normalized (landmark idx 0)
        gesture_name: str | None = None
        confidence = 0.0
        if result.gestures and result.gestures[0]:
            cat = result.gestures[0][0]
            gesture_name = str(cat.category_name)
            confidence = float(cat.score)

        # handedness — 첫 손의 anatomical label ('Left' or 'Right'); 미감지 시 None
        handedness_name: str | None = None
        if result.handedness and result.handedness[0]:
            handedness_name = str(result.handedness[0][0].category_name)

        wrist = result.hand_landmarks[0][0]    # NormalizedLandmark, x/y in [0,1]
        wrist_x = float(wrist.x)
        wrist_y = float(wrist.y)

        # wave detector 갱신 (visibility 유지하는 한 매 frame)
        self._wave.update(wrist_x)
        is_waving = self._wave.is_waving()

        event_type = classify_hand_state(
            gesture=gesture_name,
            wrist_y_normalized=wrist_y,
            is_waving=is_waving,
            handedness=handedness_name,
            up_threshold=self._up_threshold,
        )
        if event_type is None:
            return

        # per-event_type cooldown — 다른 event_type 즉시 OK, 같은 type 만 막음
        t_now = time.time()
        if (event_type == self._last_emit_type
                and (t_now - self._last_emit_t) < self._cooldown):
            return
        self._last_emit_type = event_type
        self._last_emit_t = t_now

        payload = {
            'ts': t_now,
            'event_type': event_type,
            'confidence': confidence,
            'source': 'mediapipe.GestureRecognizer',
            'gesture': gesture_name,
            'wrist_y': wrist_y,
            'is_waving': is_waving,
            'handedness': handedness_name,
        }
        out = String()
        out.data = json.dumps(payload)
        self._pub.publish(out)
        self.get_logger().info(
            f'▶ {event_type} (gesture={gesture_name}, conf={confidence:.2f}, '
            f'wrist_y={wrist_y:.2f}, waving={is_waving})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GestureDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
