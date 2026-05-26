"""dashboard_node — /emotion/state, /rapport/event, /omx_reactor/state read-only 구독.

FastAPI(HTTP) + WebSocket. 두 WS endpoint:

  /ws/v1/engaging  — doby opserver 패턴 그대로 (5Hz throttle, snapshot 통째).
                     vendored engaging-analytics.js 가 직접 구독.
  /ws/stream       — omx 고유. reactor 모션 + 이벤트 push (Step B 이벤트 타임라인 source).

emotion / rapport / engagement / minigame snapshot 함수는 doby_controller 의
moca_opserver.opserver_node 의 동일 함수와 같은 shape (engaging-analytics.js
0-modification 작동을 위한 계약).
"""
from __future__ import annotations

import asyncio
import json
import threading
import time
from collections import deque
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from dobi_npc_msgs.msg import EmotionState, RapportEvent

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
import uvicorn


# ─── doby_controller opserver helper (0 modification) ──────────
# spec: docs/superpowers/specs/2026-05-21-engagement-timeline-design.md §4
_MARKER_ELIGIBLE_TYPES = frozenset(
    {'engagement_up', 'engagement_down', 'abort_trigger'})


def compute_engagement_ema(prev: float, weight: float, alpha: float) -> float:
    """RapportEvent.weight 의 EMA — score = alpha*weight + (1-alpha)*prev."""
    return alpha * weight + (1.0 - alpha) * prev


def should_reset_engagement_score(last: int, current: int) -> bool:
    """track_id 변경 시 engagement_score cold start 여부."""
    if current == -1:
        return False
    if last == -1:
        return False
    return last != current


def is_marker_eligible(event_type: str) -> bool:
    """engagement_up / engagement_down / abort_trigger 만 marker append."""
    return event_type in _MARKER_ELIGIBLE_TYPES


def _static_dir() -> Path:
    share = get_package_share_directory('omx_reactor')
    return Path(share) / 'web' / 'static'


class DashboardNode(Node):

    def __init__(self):
        super().__init__('omx_dashboard_node')
        self.declare_parameter('http_port', 8800)
        self.declare_parameter('engagement_score_alpha', 0.1)
        self.declare_parameter('mjpeg_input_topic', '/external_cam/image')
        self._port = int(self.get_parameter('http_port').value)
        self._score_alpha = float(self.get_parameter('engagement_score_alpha').value)
        self._mjpeg_topic = str(self.get_parameter('mjpeg_input_topic').value)

        # doby opserver 와 동일 state — engaging snapshot 직접 채움
        self._emotion_state: dict | None = None
        self._emotion_history: deque = deque(maxlen=600)
        self._rapport_events: deque = deque(maxlen=20)
        self._rapport_counters: dict[str, int] = {
            'engagement_up': 0,
            'engagement_down': 0,
            'abort_trigger': 0,
            'neutral_continue': 0,
        }
        self._engagement_score: float = 0.0
        self._engagement_score_history: deque = deque(maxlen=600)
        self._rapport_marker_history: deque = deque(maxlen=30)
        self._last_score_track_id: int = -1

        # omx 고유 — /ws/stream + /api/snapshot 용 (Step B 이벤트 타임라인 source)
        self._omx_snapshot = {
            'emotion': None,
            'rapport': None,
            'reactor': None,
            'events': deque(maxlen=20),
        }
        # rclpy.Node._clients 와 충돌 회피 — final review 가 발견한 shadow bug
        self._ws_stream_clients: set[WebSocket] = set()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._loop_ready = threading.Event()

        # OMX 카메라 view — mjpeg_input_topic 구독 + 최신 JPEG 캐시 (MJPEG stream 용)
        # sim → /external_cam/image (Gazebo), real → /webcam/image_raw (uvc)
        self._cv_bridge = CvBridge()
        self._latest_jpeg: bytes | None = None
        self._jpeg_lock = threading.Lock()

        self.create_subscription(EmotionState, '/emotion/state', self._on_emotion, 10)
        self.create_subscription(RapportEvent, '/rapport/event', self._on_rapport, 10)
        self.create_subscription(String, '/omx_reactor/state', self._on_reactor, 10)
        self.create_subscription(Image, self._mjpeg_topic, self._on_cam_image, 1)
        self.get_logger().info(f'dashboard MJPEG 입력 토픽: {self._mjpeg_topic}')

        self._app = self._build_app()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        self._loop_ready.wait(timeout=5.0)
        self.get_logger().info(
            f'omx_dashboard_node ready — http://localhost:{self._port}/  '
            f'(WS: /ws/stream + /ws/v1/engaging)'
        )

    # ── FastAPI ──────────────────────────────────────────
    def _build_app(self) -> FastAPI:
        app = FastAPI(title='omx_reactor dashboard')
        static = _static_dir()

        @app.get('/')
        async def index():
            return FileResponse(static / 'index.html')

        @app.get('/api/snapshot')
        async def snapshot():
            return JSONResponse({
                'emotion': self._omx_snapshot['emotion'],
                'rapport': self._omx_snapshot['rapport'],
                'reactor': self._omx_snapshot['reactor'],
                'events': list(self._omx_snapshot['events']),
            })

        @app.websocket('/ws/v1/engaging')
        async def ws_engaging(ws: WebSocket):
            """engaging-analytics 라이브 stream — 5Hz throttle.

            doby opserver 의 ws_engaging 와 동일 패턴 + 동일 payload shape.
            vendored engaging-analytics.js 가 hack 없이 그대로 구독.
            """
            await ws.accept()
            try:
                while True:
                    payload = {
                        'ts': time.time(),
                        'emotion': self.emotion_snapshot(),
                        'rapport': self.rapport_snapshot(),
                        'engagement': self.engagement_snapshot(),
                        'minigame': self.minigame_snapshot(),
                        'mode': {
                            'current': 'engaging',
                            'entered_at': 0,
                        },
                    }
                    await ws.send_json(payload)
                    await asyncio.sleep(0.2)
            except WebSocketDisconnect:
                pass
            except Exception as e:
                self.get_logger().warning(f'ws_engaging error: {e}')

        @app.websocket('/ws/stream')
        async def ws_stream(ws: WebSocket):
            """omx 고유 — reactor 모션 + 이벤트 (Step B 의 타임라인 source)."""
            await ws.accept()
            self._ws_stream_clients.add(ws)
            try:
                await ws.send_text(json.dumps({
                    'type': 'snapshot',
                    'payload': {
                        'emotion': self._omx_snapshot['emotion'],
                        'rapport': self._omx_snapshot['rapport'],
                        'reactor': self._omx_snapshot['reactor'],
                        'events': list(self._omx_snapshot['events']),
                    },
                }))
                while True:
                    await ws.receive_text()
            except WebSocketDisconnect:
                pass
            finally:
                self._ws_stream_clients.discard(ws)

        @app.get('/api/gazebo_view.mjpg')
        async def gazebo_view():
            """OMX Gazebo 외부 카메라 → MJPEG stream (multipart/x-mixed-replace).

            sensor publish ~15Hz. 본 stream 도 ~15 FPS (5ms idle sleep + 60ms cap).
            """
            async def gen():
                boundary = b'--frame'
                last_sent = None
                while True:
                    with self._jpeg_lock:
                        jpg = self._latest_jpeg
                    if jpg is not None and jpg is not last_sent:
                        last_sent = jpg
                        yield boundary + b'\r\nContent-Type: image/jpeg\r\nContent-Length: ' \
                              + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n'
                    await asyncio.sleep(0.05)
            return StreamingResponse(gen(),
                                     media_type='multipart/x-mixed-replace; boundary=frame')

        app.mount('/static', StaticFiles(directory=static, follow_symlink=True), name='static')
        return app

    def _serve(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop_ready.set()
        config = uvicorn.Config(self._app, host='0.0.0.0', port=self._port,
                                log_level='warning', loop='asyncio')
        server = uvicorn.Server(config)
        self._loop.run_until_complete(server.serve())

    # ── doby opserver snapshot 함수 (shape 0-modification) ────
    def emotion_snapshot(self) -> dict:
        latest = dict(self._emotion_state) if self._emotion_state else None
        traj = list(self._emotion_history)
        return {
            'latest': latest,
            'trajectory': [
                {'t': round(ts, 3), 'v': round(v, 3), 'a': round(a, 3),
                 'conf': round(c, 3), 'source': s}
                for (ts, v, a, c, s) in traj
            ],
        }

    def rapport_snapshot(self) -> dict:
        return {
            'counters': dict(self._rapport_counters),
            'recent': [
                {'type': r['type'], 'weight': round(r['weight'], 2),
                 'v': round(r['v'], 2), 'a': round(r['a'], 2),
                 'conf': round(r['conf'], 2),
                 'reason': r['reason'],
                 'track_id': r.get('track_id'),
                 'group_id': r.get('group_id'),
                 'ts': round(r['ts'], 3)}
                for r in list(self._rapport_events)
            ],
        }

    def engagement_snapshot(self) -> dict:
        return {
            'score': round(self._engagement_score, 4),
            'score_history': list(self._engagement_score_history),
            'rapport_markers': list(self._rapport_marker_history),
        }

    def minigame_snapshot(self) -> dict:
        # omx_reactor 는 minigame 없음 — engaging-analytics.js 가 정상 작동하기 위한 placeholder
        return {'latest': None, 'recent': []}

    # ── ROS 구독 핸들러 — doby opserver 패턴 그대로 ────────
    def _on_emotion(self, msg: EmotionState):
        rec = {
            'v': float(msg.valence),
            'a': float(msg.arousal),
            'conf': float(msg.confidence),
            'source': msg.source,
            'flags': list(msg.flags),
            'track_id': int(msg.track_id),
            'group_id': int(msg.group_id),
            'ts': time.time(),
        }
        self._emotion_state = rec
        self._emotion_history.append(
            (rec['ts'], rec['v'], rec['a'], rec['conf'], rec['source']))
        # omx 고유 snapshot 도 갱신 (Step B 타임라인 source)
        self._omx_snapshot['emotion'] = {
            'v': rec['v'], 'a': rec['a'], 'confidence': rec['conf'],
            'source': rec['source'], 'track_id': rec['track_id'],
        }
        self._push_stream({'type': 'emotion', 'payload': self._omx_snapshot['emotion']})

    def _on_rapport(self, msg: RapportEvent):
        rec = {
            'type': msg.event_type,
            'weight': float(msg.weight),
            'v': float(msg.emotion.valence),
            'a': float(msg.emotion.arousal),
            'conf': float(msg.emotion.confidence),
            'reason': msg.reason,
            'track_id': int(msg.emotion.track_id),
            'group_id': int(msg.emotion.group_id),
            'ts': time.time(),
        }
        self._rapport_events.append(rec)

        # engagement_score EMA — doby Track C 로직 그대로
        tid = int(msg.emotion.track_id)
        if should_reset_engagement_score(self._last_score_track_id, tid):
            self.get_logger().info(
                f'engagement_score cold start: track_id '
                f'{self._last_score_track_id} → {tid}')
            self._engagement_score = 0.0
        if tid != -1:
            self._last_score_track_id = tid

        self._engagement_score = compute_engagement_ema(
            self._engagement_score, float(msg.weight), self._score_alpha)

        now = time.time()
        self._engagement_score_history.append({
            'ts': round(now, 3),
            'score': round(self._engagement_score, 4),
        })
        if is_marker_eligible(msg.event_type):
            self._rapport_marker_history.append({
                'ts': round(now, 3),
                'type': msg.event_type,
                'weight': round(float(msg.weight), 2),
            })

        if msg.event_type in self._rapport_counters:
            self._rapport_counters[msg.event_type] += 1

        # omx 고유 snapshot 도 갱신 (Step B 타임라인 source)
        ev = {
            'event_type': str(msg.event_type),
            'weight': float(msg.weight),
            'reason': str(msg.reason),
            'v': float(msg.emotion.valence),
            'a': float(msg.emotion.arousal),
        }
        self._omx_snapshot['rapport'] = ev
        self._omx_snapshot['events'].append({'type': 'rapport', **ev})
        self._push_stream({'type': 'rapport', 'payload': ev})

    def _on_cam_image(self, msg: Image):
        """Gazebo 외부 카메라 frame 받아 JPEG 인코딩 후 캐시."""
        try:
            bgr = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            ok, jpg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with self._jpeg_lock:
                    self._latest_jpeg = jpg.tobytes()
        except Exception as e:
            self.get_logger().warning(f'gazebo cam encode 실패: {e}')

    def _on_reactor(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        prev = self._omx_snapshot['reactor']
        self._omx_snapshot['reactor'] = payload
        prev_motion = prev.get('current_motion') if prev else None
        cur_motion = payload.get('current_motion')
        if cur_motion is not None and prev_motion != cur_motion:
            self._omx_snapshot['events'].append({'type': 'motion', **payload})
        self._push_stream({'type': 'reactor', 'payload': payload})

    def _push_stream(self, message: dict):
        if self._loop is None or not self._ws_stream_clients:
            return
        text = json.dumps(message)
        for ws in list(self._ws_stream_clients):
            asyncio.run_coroutine_threadsafe(self._send_safe(ws, text), self._loop)

    async def _send_safe(self, ws: WebSocket, text: str):
        try:
            await ws.send_text(text)
        except Exception:
            self._ws_stream_clients.discard(ws)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
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
